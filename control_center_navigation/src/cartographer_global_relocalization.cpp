#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <deque>
#include <map>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/utils.h"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double angular_distance(double lhs, double rhs)
{
  return std::abs(normalize_angle(lhs - rhs));
}

struct ScanPoint
{
  double x{0.0};
  double y{0.0};
  double weight{1.0};
  double tangent{0.0};
  bool line{false};
  double origin_x{0.0}, origin_y{0.0};
  bool origin_valid{false};
  bool near_return{false};
};

struct RotatedOffset
{
  double x_cells{0.0};
  double y_cells{0.0};
  double weight{1.0};
  double tangent{0.0};
  bool line{false};
  int sector{0};
};

struct Candidate
{
  double cell_x{0.0};
  double cell_y{0.0};
  double yaw_in_map{0.0};
  double score{0.0};
};

struct MatchQuality
{
  double score{0.0};
  double matched_ratio{0.0};
  double structural_ratio{0.0};
  int sectors{0};
  int orientations{0};
  double normal_diversity{0.0};
};

}  // namespace

class CartographerGlobalRelocalization : public rclcpp::Node
{
public:
  CartographerGlobalRelocalization()
  : Node("cartographer_global_relocalization"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    scan_topic_ = declare_parameter<std::string>(
      "scan_topic", "/topic_gv_front_lidar_0_307");
    rear_scan_topic_ = declare_parameter<std::string>(
      "rear_scan_topic", "/topic_gv_rear_lidar_0_307");
    initialpose_topic_ = declare_parameter<std::string>("initialpose_topic", "/initialpose");
    result_pose_topic_ = declare_parameter<std::string>(
      "result_pose_topic", "/global_relocalization/result_pose");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/global_relocalization/status");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");

    auto_start_ = declare_parameter<bool>("auto_start", true);
    publish_initial_pose_ = declare_parameter<bool>("publish_initial_pose", true);
    use_rear_scan_ = declare_parameter<bool>("use_rear_scan", true);
    require_rear_scan_ = declare_parameter<bool>("require_rear_scan", true);
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 65);
    free_threshold_ = declare_parameter<int>("free_threshold", 25);
    min_scan_points_ = declare_parameter<int>("min_scan_points", 60);
    coarse_scan_points_ = declare_parameter<int>("coarse_scan_points", 90);
    fine_scan_points_ = declare_parameter<int>("fine_scan_points", 240);
    top_candidates_ = declare_parameter<int>("top_candidates", 40);
    max_attempts_ = declare_parameter<int>("max_attempts", 8);
    publish_repetitions_ = declare_parameter<int>("publish_repetitions", 3);
    required_consistent_matches_ =
      declare_parameter<int>("required_consistent_matches", 3);

    min_range_ = declare_parameter<double>("min_range", 0.20);
    max_range_ = declare_parameter<double>("max_range", 15.0);
    coarse_linear_step_ = declare_parameter<double>("coarse_linear_step", 0.25);
    coarse_angular_step_deg_ = declare_parameter<double>("coarse_angular_step_deg", 5.0);
    fine_linear_window_ = declare_parameter<double>("fine_linear_window", 0.35);
    fine_linear_step_ = declare_parameter<double>("fine_linear_step", 0.05);
    fine_angular_window_deg_ = declare_parameter<double>("fine_angular_window_deg", 6.0);
    fine_angular_step_deg_ = declare_parameter<double>("fine_angular_step_deg", 1.0);
    likelihood_sigma_ = declare_parameter<double>("likelihood_sigma", 0.12);
    max_obstacle_distance_ = declare_parameter<double>("max_obstacle_distance", 0.75);
    robot_clearance_ = declare_parameter<double>("robot_clearance", 0.20);
    min_score_ = declare_parameter<double>("min_score", 0.52);
    near_wall_matching_ = declare_parameter<bool>("near_wall_matching", true);
    supplemental_verification_ = declare_parameter<bool>("supplemental_verification", true);
    supplemental_margin_ = declare_parameter<double>("supplemental_margin", 0.06);
    maximum_wall_conflict_ = declare_parameter<double>("maximum_wall_conflict", 0.15);
    if (supplemental_margin_ < 0.03 || supplemental_margin_ > 1.0 ||
      maximum_wall_conflict_ < 0.0 || maximum_wall_conflict_ > 0.25) {
      throw std::runtime_error("invalid supplemental verification parameters");
    }
    min_score_margin_ = declare_parameter<double>("min_score_margin", 0.03);
    scan_pair_max_age_sec_ = declare_parameter<double>("scan_pair_max_age_sec", 0.30);
    scan_max_age_sec_ = declare_parameter<double>("scan_max_age_sec", 0.60);
    minimum_matched_ratio_ = declare_parameter<double>("minimum_matched_ratio", 0.35);
    minimum_structural_ratio_ = declare_parameter<double>("minimum_structural_ratio", 0.15);
    minimum_matched_sectors_ = declare_parameter<int>("minimum_matched_sectors", 3);
    minimum_normal_diversity_ = declare_parameter<double>("minimum_normal_diversity", 0.06);
    occluded_point_weight_ = declare_parameter<double>("occluded_point_weight", 0.45);
    candidate_region_m_ = declare_parameter<double>("candidate_region_m", 1.0);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    consistency_position_tolerance_ =
      declare_parameter<double>("consistency_position_tolerance", 0.25);
    consistency_yaw_tolerance_deg_ =
      declare_parameter<double>("consistency_yaw_tolerance_deg", 5.0);
    retry_interval_sec_ = declare_parameter<double>("retry_interval_sec", 1.0);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.30);
    covariance_x_ = declare_parameter<double>("covariance_x", 0.25);
    covariance_y_ = declare_parameter<double>("covariance_y", 0.25);
    covariance_yaw_ = declare_parameter<double>("covariance_yaw", 0.20);
    diagnostic_reference_enabled_ =
      declare_parameter<bool>("diagnostic_reference_enabled", false);
    diagnostic_reference_x_ = declare_parameter<double>("diagnostic_reference_x", 0.0);
    diagnostic_reference_y_ = declare_parameter<double>("diagnostic_reference_y", 0.0);
    diagnostic_reference_yaw_ = declare_parameter<double>("diagnostic_reference_yaw", 0.0);

    if (coarse_linear_step_ <= 0.0 || fine_linear_step_ <= 0.0) {
      throw std::runtime_error("linear search steps must be positive");
    }
    if (coarse_angular_step_deg_ <= 0.0 || fine_angular_step_deg_ <= 0.0) {
      throw std::runtime_error("angular search steps must be positive");
    }
    if (likelihood_sigma_ <= 0.0 || max_obstacle_distance_ <= 0.0) {
      throw std::runtime_error("likelihood parameters must be positive");
    }
    if (scan_pair_max_age_sec_ <= 0.0 || scan_max_age_sec_ <= 0.0 ||
      top_candidates_ < 2 || coarse_scan_points_ < 8 || fine_scan_points_ < 8 ||
      candidate_region_m_ < coarse_linear_step_ || minimum_matched_sectors_ < 2 ||
      minimum_matched_sectors_ > 8 || minimum_normal_diversity_ <= 0.0 ||
      minimum_normal_diversity_ > 0.5 || minimum_matched_ratio_ <= 0.0 ||
      minimum_matched_ratio_ > 1.0 || minimum_structural_ratio_ < 0.0 ||
      minimum_structural_ratio_ > 1.0 || occluded_point_weight_ < 0.25 ||
      occluded_point_weight_ > 1.0)
    {
      throw std::runtime_error("invalid robust relocalization parameters");
    }

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&CartographerGlobalRelocalization::map_callback, this, std::placeholders::_1));
    scan_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions scan_options;
    scan_options.callback_group = scan_group_;
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_callback(msg, false);
      }, scan_options);
    if (use_rear_scan_) {
      rear_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        rear_scan_topic_,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          scan_callback(msg, true);
        }, scan_options);
    }
    initialpose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, rclcpp::QoS(1).reliable().transient_local());
    result_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      result_pose_topic_, rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(10).reliable().transient_local());
    retry_service_ = create_service<std_srvs::srv::Trigger>(
      "~/relocalize",
      std::bind(
        &CartographerGlobalRelocalization::retry_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    enabled_ = auto_start_;
    search_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {search_latest_pair();});
    publish_status("waiting", "waiting for map and laser scan");
    RCLCPP_INFO(
      get_logger(),
      "Cartographer-style global relocalization ready: map=%s front=%s rear=%s "
      "use_rear=%s require_rear=%s auto_start=%s",
      map_topic_.c_str(), scan_topic_.c_str(), rear_scan_topic_.c_str(),
      use_rear_scan_ ? "true" : "false", require_rear_scan_ ? "true" : "false",
      auto_start_ ? "true" : "false");
  }

private:
  friend struct RelocalizationTest;
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (msg->info.width == 0 || msg->info.height == 0 || msg->info.resolution <= 0.0 ||
      msg->data.size() != static_cast<size_t>(msg->info.width) * msg->info.height) {
      RCLCPP_ERROR(get_logger(), "Rejected invalid occupancy grid");
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    map_ = *msg;
    supplemental_unique_=false; balanced_margin_=0.0; wall_conflict_=0.0;
    have_consistency_candidate_ = false;
    consistent_match_count_ = 0;
    map_origin_yaw_ = tf2::getYaw(map_.info.origin.orientation);
    build_distance_field();
    map_ready_ = true;
    publish_status("map_ready", "map received; waiting for a valid laser scan");
    RCLCPP_INFO(
      get_logger(), "Map prepared: %ux%u resolution=%.3f occupied_cells=%zu",
      map_.info.width, map_.info.height, map_.info.resolution, occupied_cell_count_);
  }

  void build_distance_field()
  {
    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    const float infinity = std::numeric_limits<float>::max() / 8.0F;
    const float diagonal = std::sqrt(2.0F);
    distance_cells_.assign(static_cast<std::size_t>(width * height), infinity);
    nearest_occupied_.assign(distance_cells_.size(), -1);
    map_structure_.assign(distance_cells_.size(), 1.0F);
    map_tangent_.assign(distance_cells_.size(), std::numeric_limits<float>::quiet_NaN());
    occupied_cell_count_ = 0;

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const std::size_t index = static_cast<std::size_t>(y * width + x);
        if (map_.data[index] >= occupied_threshold_) {
          distance_cells_[index] = 0.0F;
          nearest_occupied_[index] = static_cast<int>(index);
          ++occupied_cell_count_;
        }
      }
    }

    const auto relax = [this, width, height](
      int x, int y, int neighbor_x, int neighbor_y, float cost)
      {
        if (neighbor_x < 0 || neighbor_y < 0 || neighbor_x >= width || neighbor_y >= height) {
          return;
        }
        const std::size_t index = static_cast<std::size_t>(y * width + x);
        const std::size_t neighbor =
          static_cast<std::size_t>(neighbor_y * width + neighbor_x);
        if (distance_cells_[neighbor] + cost < distance_cells_[index]) {
          distance_cells_[index] = distance_cells_[neighbor] + cost;
          nearest_occupied_[index] = nearest_occupied_[neighbor];
        }
      };

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        relax(x, y, x - 1, y, 1.0F);
        relax(x, y, x, y - 1, 1.0F);
        relax(x, y, x - 1, y - 1, diagonal);
        relax(x, y, x + 1, y - 1, diagonal);
      }
    }
    for (int y = height - 1; y >= 0; --y) {
      for (int x = width - 1; x >= 0; --x) {
        relax(x, y, x + 1, y, 1.0F);
        relax(x, y, x, y + 1, 1.0F);
        relax(x, y, x + 1, y + 1, diagonal);
        relax(x, y, x - 1, y + 1, diagonal);
      }
    }

    const double resolution = map_.info.resolution;
    // Local occupied-neighbour support: walls/corners/column outlines get a
    // bounded boost. Isolated chair legs remain present with weight 1, not deleted.
    const int radius = std::max(2, static_cast<int>(std::ceil(0.15 / resolution)));
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int index = y * width + x;
        if (map_.data[index] < occupied_threshold_) {continue;}
        double sx=0, sy=0, xx=0, yy=0, xy=0, count=0;
        for (int dy=-radius; dy<=radius; ++dy) {
          for (int dx=-radius; dx<=radius; ++dx) {
            if (x+dx<0 || y+dy<0 || x+dx>=width || y+dy>=height ||
              map_.data[(y+dy)*width+x+dx] < occupied_threshold_) {continue;}
            ++count; sx+=dx; sy+=dy; xx+=dx*dx; yy+=dy*dy; xy+=dx*dy;
          }
        }
        if (count < 4) {continue;}
        xx=xx/count-(sx/count)*(sx/count); yy=yy/count-(sy/count)*(sy/count);
        xy=xy/count-sx*sy/(count*count);
        if ((xx+yy)*resolution*resolution < 0.003) {continue;}
        map_structure_[index] = 1.4F;
        if (std::hypot(xx-yy,2*xy) / std::max(1e-9,xx+yy) > 0.65) {
          map_tangent_[index] = static_cast<float>(0.5*std::atan2(2*xy,xx-yy));
        }
      }
    }
    const int lut_size =
      std::max(2, static_cast<int>(std::ceil(max_obstacle_distance_ / resolution)) + 2);
    likelihood_lut_.resize(static_cast<std::size_t>(lut_size));
    for (int cell = 0; cell < lut_size; ++cell) {
      const double distance = cell * resolution;
      likelihood_lut_[static_cast<std::size_t>(cell)] =
        static_cast<float>(
        std::exp(-0.5 * distance * distance / (likelihood_sigma_ * likelihood_sigma_)));
    }
  }

  void scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg, bool is_rear_scan)
  {
    // This callback never runs the search or waits for TF. Its independent
    // executor group keeps bounded, timestamp-ordered caches fresh during search.
    const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
    if (stamp <= 0) {return;}
    std::lock_guard<std::mutex> lock(scan_mutex_);
    auto & cache = is_rear_scan ? rear_scans_ : front_scans_;
    if (!cache.empty() && stamp <= rclcpp::Time(cache.back()->header.stamp).nanoseconds()) {
      return;  // duplicate / out-of-order acquisition, not a new observation
    }
    cache.push_back(msg);
    while (cache.size() > 8) {cache.pop_front();}
  }

  bool select_scan_pair(sensor_msgs::msg::LaserScan::SharedPtr & front,
    sensor_msgs::msg::LaserScan::SharedPtr & rear)
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    const double ros_now = this->now().seconds();
    const auto fresh = [&](const auto & scan) {
        const double age = ros_now - rclcpp::Time(scan->header.stamp).seconds();
        return age >= -0.05 && age <= scan_max_age_sec_;
      };
    for (auto f = front_scans_.rbegin(); f != front_scans_.rend(); ++f) {
      const auto ft = rclcpp::Time((*f)->header.stamp);
      if (!fresh(*f) || ft.nanoseconds() <= last_front_stamp_) {continue;}
      double delta = scan_pair_max_age_sec_;
      if (use_rear_scan_) {
        for (auto r = rear_scans_.rbegin(); r != rear_scans_.rend(); ++r) {
          const auto rt = rclcpp::Time((*r)->header.stamp);
          if (!fresh(*r) || rt.nanoseconds() <= last_rear_stamp_) {continue;}
          const double difference = std::abs((ft - rt).seconds());
          if (difference <= delta) {rear = *r; delta = difference;}
        }
      }
      if (rear || !use_rear_scan_ || !require_rear_scan_) {front = *f; return true;}
    }
    return false;
  }

  std::string scan_wait_detail()
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    if (front_scans_.empty() && use_rear_scan_ && rear_scans_.empty()) {
      return "前后雷达均无有效数据，请检查雷达采集服务；数据恢复后自动继续";
    }
    if (front_scans_.empty()) {
      return "前雷达无有效数据，请检查前雷达及采集服务；数据恢复后自动继续";
    }
    if (use_rear_scan_ && require_rear_scan_ && rear_scans_.empty()) {
      return "后雷达无有效数据，请检查后雷达及采集服务；数据恢复后自动继续";
    }
    const double now_sec = this->now().seconds();
    const auto age = [&](const auto & scans) {
        return now_sec - rclcpp::Time(scans.back()->header.stamp).seconds();
      };
    const double front_age = age(front_scans_);
    const bool rear_required = use_rear_scan_ && require_rear_scan_;
    const double rear_age = rear_required ? age(rear_scans_) : 0.0;
    std::ostringstream detail;
    detail.precision(3);
    detail << std::fixed;
    if (front_age < -0.05 || (rear_required && rear_age < -0.05)) {
      detail << "雷达时间戳超前，请检查系统校时";
    } else if (front_age > scan_max_age_sec_ ||
      (rear_required && rear_age > scan_max_age_sec_)) {
      detail << "雷达数据已过期，请检查采集是否中断";
    } else {
      double closest = std::numeric_limits<double>::infinity();
      for (const auto & f : front_scans_) {
        const auto ft = rclcpp::Time(f->header.stamp);
        if (now_sec-ft.seconds() < -0.05 || now_sec-ft.seconds() > scan_max_age_sec_) {
          continue;
        }
        for (const auto & r : rear_scans_) {
          const auto rt = rclcpp::Time(r->header.stamp);
          if (now_sec-rt.seconds() < -0.05 || now_sec-rt.seconds() > scan_max_age_sec_) {
            continue;
          }
          closest = std::min(closest, std::abs((ft-rt).seconds()));
        }
      }
      if (rear_required && closest > scan_pair_max_age_sec_) {
        detail << "前后雷达采集时间差过大，最小差=" << closest << "秒";
      } else {
        detail << "等待下一组新鲜雷达数据";
      }
    }
    detail << "；前雷达数据龄=" << front_age << "秒";
    if (rear_required) {detail << "，后雷达数据龄=" << rear_age << "秒";}
    return detail.str();
  }

  void search_latest_pair()
  {
    if (!enabled_ || completed_ || !map_ready_) {return;}
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_attempt_time_).count() < retry_interval_sec_) {
      return;
    }
    sensor_msgs::msg::LaserScan::SharedPtr front, rear;
    if (!select_scan_pair(front, rear)) {
      have_consistency_candidate_ = false;
      consistent_match_count_ = 0;
      const auto detail = scan_wait_detail();
      publish_status("waiting_scans", detail);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", detail.c_str());
      return;
    }
    const rclcpp::Time reference(front->header.stamp);
    std::vector<ScanPoint> combined_points, rear_points;
    // Both lasers are transformed into base_link at the SAME front timestamp,
    // using odom only for temporal alignment (not a historical map prior).
    if (!laser_scan_in_base(*front, combined_points, reference) ||
      (rear && !laser_scan_in_base(*rear, rear_points, reference))) {
      have_consistency_candidate_=false; consistent_match_count_=0; return;
    }
    combined_points.insert(combined_points.end(), rear_points.begin(), rear_points.end());
    if (static_cast<int>(combined_points.size()) < min_scan_points_) {
      have_consistency_candidate_=false; consistent_match_count_=0; return;
    }
    last_front_stamp_ = reference.nanoseconds();
    if (rear) {last_rear_stamp_ = rclcpp::Time(rear->header.stamp).nanoseconds();}

    matching_ = true;
    last_attempt_time_ = now;
    ++attempt_count_;
    publish_status("matching", "running full-map correlative scan matching");
    const auto started = std::chrono::steady_clock::now();

    Candidate best;
    Candidate second;
    bool found = false;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      found = match(combined_points, best, second);
    }

    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    matching_ = false;

    const double score_margin = best.score - second.score;
    const bool unique_match =
      second.score <= 0.0 || score_margin >= min_score_margin_ || supplemental_unique_;
    bool stationary = false;
    try {
      const auto motion = tf_buffer_.lookupTransform(base_frame_, this->now(),
        base_frame_, reference, odom_frame_, rclcpp::Duration::from_seconds(transform_timeout_sec_));
      stationary = std::hypot(motion.transform.translation.x, motion.transform.translation.y) < 0.04 &&
        std::abs(tf2::getYaw(motion.transform.rotation)) < 0.03;
    } catch (const tf2::TransformException &) {}
    if (found && best.score >= min_score_ && unique_match && stationary && near_mode_valid_ && quality_valid(last_quality_)) {
      const auto pose = candidate_to_pose(best);
      if (required_consistent_matches_ > 1) {
        bool agrees_with_previous = false;
        if (have_consistency_candidate_) {
          const auto previous_pose = candidate_to_pose(consistency_candidate_);
          agrees_with_previous =
            std::hypot(pose[0] - previous_pose[0], pose[1] - previous_pose[1]) <=
            consistency_position_tolerance_ &&
            angular_distance(pose[2], previous_pose[2]) <=
            consistency_yaw_tolerance_deg_ * kPi / 180.0;
        }
        if (agrees_with_previous) {
          ++consistent_match_count_;
        } else {
          consistent_match_count_ = 1;
        }
        consistency_candidate_ = best;
        have_consistency_candidate_ = true;
        if (consistent_match_count_ < required_consistent_matches_) {
          RCLCPP_INFO(
            get_logger(),
            "Global candidate confirmation %d/%d: score=%.4f margin=%.4f "
            "pose=[%.3f, %.3f, %.3f]",
            consistent_match_count_, required_consistent_matches_, best.score, score_margin,
            pose[0], pose[1], pose[2]);
          publish_status(
            "confirming", "candidate accepted; waiting for consistent scans",
            best.score, elapsed);
          if (max_attempts_ > 0 && attempt_count_ >= max_attempts_) {
            enabled_ = false;
            publish_status(
              "failed", "maximum relocalization attempts reached before confirmation",
              best.score, elapsed);
            RCLCPP_ERROR(
              get_logger(),
              "Global relocalization stopped after %d attempts; call ~/relocalize to retry",
              attempt_count_);
          }
          return;
        }
      }
      RCLCPP_INFO(
        get_logger(),
        "Global relocalization succeeded in %.3fs: score=%.4f second=%.4f margin=%.4f "
        "pose=[%.3f, %.3f, %.3f]",
        elapsed, best.score, second.score, score_margin, pose[0], pose[1], pose[2]);
      publish_result(pose[0], pose[1], pose[2], best.score);
      completed_ = true;
      publish_status("succeeded", publish_initial_pose_ ? "global pose published to AMCL" :
        "diagnostic match succeeded; AMCL publication disabled", best.score, elapsed);
      return;
    }

    have_consistency_candidate_ = false;
    consistent_match_count_ = 0;  // confirmations must be consecutive independent scans
    RCLCPP_WARN(
      get_logger(),
      "Global relocalization attempt %d rejected in %.3fs: best=%.4f second=%.4f "
      "margin=%.4f (required score>=%.4f margin>=%.4f)",
      attempt_count_, elapsed, best.score, second.score, score_margin,
      min_score_, min_score_margin_);
    publish_status(
      "retrying",
      !stationary ? "keep robot stationary; odometry missing or robot moved during search" :
      !near_mode_valid_ ? "near-wall candidate has contradictory visibility or distance evidence" :
      !quality_valid(last_quality_) ? "stable structures insufficient; expose more wall corners and retry" :
      best.score < min_score_ ? "match score below threshold" :
      "ambiguous match; best and second candidates are too close",
      best.score, elapsed);
    if (max_attempts_ > 0 && attempt_count_ >= max_attempts_) {
      enabled_ = false;
      publish_status("failed", "maximum relocalization attempts reached", best.score, elapsed);
      RCLCPP_ERROR(
        get_logger(),
        "Global relocalization stopped after %d attempts; call ~/relocalize to retry",
        attempt_count_);
    }
  }

  bool laser_scan_in_base(
    const sensor_msgs::msg::LaserScan & scan,
    std::vector<ScanPoint> & points, const rclcpp::Time & reference)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        base_frame_, reference, scan.header.frame_id, rclcpp::Time(scan.header.stamp), odom_frame_,
        rclcpp::Duration::from_seconds(transform_timeout_sec_));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for laser-to-base transform %s <- %s: %s",
        base_frame_.c_str(), scan.header.frame_id.c_str(), error.what());
      return false;
    }

    tf2::Transform laser_to_base;
    tf2::fromMsg(transform.transform, laser_to_base);
    const double effective_min = std::max(min_range_, static_cast<double>(scan.range_min));
    const double effective_max = std::min(max_range_, static_cast<double>(scan.range_max));

    points.reserve(scan.ranges.size());
    std::vector<ScanPoint> close;
    for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
      const double range = scan.ranges[index];
      if (!std::isfinite(range) || range >= effective_max) {
        continue;
      }
      const bool near=range<=effective_min;
      if(near && !trusted_near_return(scan,index)) continue;
      const double angle = scan.angle_min + index * scan.angle_increment;
      const double laser_x = range * std::cos(angle);
      const double laser_y = range * std::sin(angle);
      const tf2::Vector3 point_in_base =
        laser_to_base * tf2::Vector3(laser_x, laser_y, 0.0);
      ScanPoint p{point_in_base.x(),point_in_base.y()};
      p.origin_x=laser_to_base.getOrigin().x();p.origin_y=laser_to_base.getOrigin().y();
      p.origin_valid=true;p.near_return=near;
      if(near) {
        // Exclude rays directed inward toward the chassis, not an assumed footprint.
        if((p.x-p.origin_x)*p.origin_x+(p.y-p.origin_y)*p.origin_y<=0) continue;
        close.push_back(p);
      } else points.push_back(p);
    }
    annotate_structure(points);
    points.insert(points.end(),close.begin(),close.end());
    return true;
  }

  bool trusted_near_return(const sensor_msgs::msg::LaserScan & scan,size_t index) const
  {
    if(!near_wall_matching_ || index<2 || index+2>=scan.ranges.size()) return false;
    const double floor=std::max(0.04,static_cast<double>(scan.range_min));
    const double r=scan.ranges[index];
    if(!std::isfinite(r) || r<=floor || r>min_range_) return false;
    for(size_t i=index-2;i<=index+2;++i) {
      const double v=scan.ranges[i];
      if(!std::isfinite(v) || v<=floor || std::abs(v-r)>=0.015) return false;
    }
    return true;
  }

  static void annotate_structure(std::vector<ScanPoint> & points)
  {
    for (size_t i = 2; i + 2 < points.size(); ++i) {
      const auto & a = points[i - 2]; const auto & b = points[i + 2];
      const double dx = b.x - a.x, dy = b.y - a.y, span = std::hypot(dx, dy);
      if (span < 0.20 || span > 0.85) {continue;}
      bool line = true;
      for (size_t j = i - 1; j <= i + 2; ++j) {
        line &= std::hypot(points[j].x - points[j-1].x, points[j].y - points[j-1].y) < 0.25;
        line &= std::abs(dx * (points[j].y-a.y) - dy * (points[j].x-a.x)) / span < 0.025;
      }
      if (line) {points[i].line = true; points[i].tangent = std::atan2(dy, dx); points[i].weight = 1.6;}
    }
  }

  std::vector<ScanPoint> uniformly_sample(
    const std::vector<ScanPoint> & input, int requested) const
  {
    if (requested <= 0 || input.size() <= static_cast<std::size_t>(requested)) {
      return input;
    }
    std::vector<ScanPoint> output;
    output.reserve(static_cast<std::size_t>(requested));
    const double stride = static_cast<double>(input.size()) / requested;
    for (int index = 0; index < requested; ++index) {
      const auto source = std::min(
        input.size() - 1,
        static_cast<std::size_t>(std::floor((index + 0.5) * stride)));
      output.push_back(input[source]);
    }
    return output;
  }

  std::vector<RotatedOffset> rotate_scan(
    const std::vector<ScanPoint> & points, double yaw_in_map) const
  {
    std::vector<RotatedOffset> offsets;
    offsets.reserve(points.size());
    const double cosine = std::cos(yaw_in_map);
    const double sine = std::sin(yaw_in_map);
    const double inverse_resolution = 1.0 / map_.info.resolution;
    for (const auto & point : points) {
      offsets.push_back(
        {
          (cosine * point.x - sine * point.y) * inverse_resolution,
          (sine * point.x + cosine * point.y) * inverse_resolution,
          point.weight, point.tangent + yaw_in_map, point.line,
          std::clamp(static_cast<int>((std::atan2(point.y, point.x)+kPi)*4/kPi),0,7)
        });
    }
    return offsets;
  }

  double score_candidate(
    double cell_x, double cell_y, const std::vector<RotatedOffset> & offsets) const
  {
    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    double score_sum = 0.0;
    for (const auto & offset : offsets) {
      const int x = static_cast<int>(std::lround(cell_x + offset.x_cells));
      const int y = static_cast<int>(std::lround(cell_y + offset.y_cells));
      if (x < 0 || y < 0 || x >= width || y >= height) {
        continue;
      }
      const std::size_t index = static_cast<std::size_t>(y * width + x);
      if (map_.data[index] < 0) {
        continue;
      }
      const float distance = distance_cells_[index];
      const int lut_index = std::min(
        static_cast<int>(likelihood_lut_.size()) - 1,
        std::max(0, static_cast<int>(std::lround(distance))));
      score_sum += likelihood_lut_[static_cast<std::size_t>(lut_index)];
    }
    return offsets.empty() ? 0.0 : score_sum / static_cast<double>(offsets.size());
  }

  double close_score(double cx, double cy, double yaw,double sigma=0.10) const
  {
    if(close_points_.empty()) return 1.0;
    const double c=std::cos(yaw),s=std::sin(yaw),res=map_.info.resolution;
    double sum=0;
    for(const auto & p:close_points_) {
      const double px=cx+(c*p.x-s*p.y)/res, py=cy+(s*p.x+c*p.y)/res;
      const int ix=static_cast<int>(std::lround(px)), iy=static_cast<int>(std::lround(py));
      if(ix<0 || iy<0 || ix>=static_cast<int>(map_.info.width) ||
        iy>=static_cast<int>(map_.info.height) || map_.data[iy*map_.info.width+ix]<0) {continue;}
      const double d=interpolated_distance(px,py);
      sum+=std::exp(-0.5*d*d/(sigma*sigma));
    }
    return sum/close_points_.size();
  }

  double ranking_score(double cx,double cy,double yaw,const std::vector<RotatedOffset> & offsets) const
  {
    const double base=evaluate_candidate(cx,cy,offsets).score;
    return close_points_.empty()?base:base*(0.65+0.35*close_score(cx,cy,yaw));
  }

  double interpolated_distance(double x, double y) const
  {
    const int w=static_cast<int>(map_.info.width), h=static_cast<int>(map_.info.height);
    const int ix=static_cast<int>(std::floor(x)), iy=static_cast<int>(std::floor(y));
    if (ix<0 || iy<0 || ix+1>=w || iy+1>=h) {return max_obstacle_distance_;}
    const double fx=x-ix, fy=y-iy;
    return map_.info.resolution * ((1-fy)*((1-fx)*distance_cells_[iy*w+ix]+
      fx*distance_cells_[iy*w+ix+1])+fy*((1-fx)*distance_cells_[(iy+1)*w+ix]+
      fx*distance_cells_[(iy+1)*w+ix+1]));
  }

  MatchQuality evaluate_candidate(double cx, double cy,
    const std::vector<RotatedOffset> & offsets) const
  {
    MatchQuality q;
    std::array<int,8> observed{}, matched{};
    double normal_xx=0, normal_xy=0, normal_yy=0;
    int normal_count=0;
    double sum=0, denominator=0; int hits=0, structural=0;
    const int w=static_cast<int>(map_.info.width), h=static_cast<int>(map_.info.height);
    for (const auto & p : offsets) {
      ++observed[p.sector];
      const double px=cx+p.x_cells, py=cy+p.y_cells;
      const int x=static_cast<int>(std::lround(px)), y=static_cast<int>(std::lround(py));
      double weight=p.weight;
      if (x<0 || y<0 || x>=w || y>=h || map_.data[y*w+x]<0) {denominator+=weight; continue;}
      const int nearest=nearest_occupied_[y*w+x];
      if (nearest<0) {denominator+=weight; continue;}
      const double d=interpolated_distance(px,py);
      const double tangent=map_tangent_[nearest];
      double alignment=1.0;
      if (p.line && std::isfinite(tangent)) {
        alignment=0.25+0.75*std::pow(std::cos(p.tangent-tangent),2);
      }
      const double likelihood=std::exp(-0.5*d*d/(likelihood_sigma_*likelihood_sigma_))*alignment;
      if (d<0.18 && alignment>0.65) {
        ++hits; ++matched[p.sector];
        if (map_structure_[nearest]>1.0) {
          ++structural;
          weight*=map_structure_[nearest];
          if (std::isfinite(tangent)) {
            const double nx=-std::sin(tangent), ny=std::cos(tangent);
            normal_xx+=nx*nx; normal_xy+=nx*ny; normal_yy+=ny*ny; ++normal_count;
          }
        }
      } else if (d>0.25 && map_.data[y*w+x]<=free_threshold_) {
        // Only discount plausible foreground occlusion: the measured endpoint
        // is in known free space, with mapped structure farther along its ray.
        const double length=std::hypot(p.x_cells,p.y_cells);
        if (length>1.0) {
          for (double step : {0.25,0.50,1.0,1.5,2.0}) {
            const double scale=step/(length*map_.info.resolution);
            const int bx=static_cast<int>(std::lround(px+p.x_cells*scale));
            const int by=static_cast<int>(std::lround(py+p.y_cells*scale));
            if (bx<0 || by<0 || bx>=w || by>=h) {break;}
            const int ni=nearest_occupied_[by*w+bx];
            if (map_.data[by*w+bx]<0) {break;}
            if (ni>=0 && map_structure_[ni]>1.0 &&
              distance_cells_[by*w+bx]*map_.info.resolution<0.15) {
              weight*=occluded_point_weight_; break;
            }
          }
        }
      }
      sum+=weight*likelihood; denominator+=weight;
    }
    if (offsets.empty()) {return q;}
    q.score=denominator>0 ? sum/denominator : 0;
    q.matched_ratio=static_cast<double>(hits)/offsets.size();
    q.structural_ratio=static_cast<double>(structural)/offsets.size();
    for (size_t i=0; i<8; ++i) {
      if (matched[i]>=3 && matched[i]>=0.30*observed[i]) {++q.sectors;}
    }
    // Continuous normal-information test, not brittle angle-bin boundaries.
    // Parallel walls have a zero minor eigenvalue even across many sectors.
    if (normal_count>=6) {
      const double trace=normal_xx+normal_yy;
      q.normal_diversity=0.5*(trace-std::hypot(normal_xx-normal_yy,2*normal_xy))/trace;
      q.orientations=q.normal_diversity>=minimum_normal_diversity_ ? 2 : 1;
    }
    return q;
  }

  bool quality_valid(const MatchQuality & q) const
  {
    return q.matched_ratio>=minimum_matched_ratio_ &&
      q.structural_ratio>=minimum_structural_ratio_ &&
      q.sectors>=minimum_matched_sectors_ && q.orientations>=2;
  }

  struct VisibilityQuality {
    double conflict_ratio{1.0};
    int rays{0}, conflicts{0}, conflict_sectors{0};
  };

  struct BalancedQuality {
    double score{0.0};
    int sectors{0};
  };

  BalancedQuality balanced_quality(const Candidate & c,
    const std::array<std::vector<ScanPoint>,8> & sectors) const
  {
    BalancedQuality result;
    for (const auto & sector:sectors) {
      if(sector.size()<6) {continue;}
      ++result.sectors;
      result.score+=evaluate_candidate(c.cell_x,c.cell_y,
        rotate_scan(sector,c.yaw_in_map)).score;
    }
    if(result.sectors>0) result.score/=result.sectors;
    return result;
  }

  bool verify_ambiguous(const std::vector<ScanPoint> & points,
    const std::vector<Candidate> & candidates)
  {
    if(candidates.size()<2 || !quality_valid(last_quality_)) {return false;}
    std::array<std::vector<ScanPoint>,8> sectors;
    for(const auto & p:points) {
      const int sector=std::clamp(static_cast<int>((std::atan2(p.y,p.x)+kPi)*4/kPi),0,7);
      sectors[sector].push_back(p);
    }
    // Bound extra work, while retaining ALL directions (not just best-scoring points).
    std::vector<ScanPoint> rays;
    for(auto & sector:sectors) {
      sector=uniformly_sample(sector,96);
      rays.insert(rays.end(),sector.begin(),sector.end());
    }
    const auto & best=candidates.front();
    const auto b=balanced_quality(best,sectors);
    const auto visibility=visibility_quality(best,rays);
    wall_conflict_=visibility.conflict_ratio;
    if(b.sectors<6 || b.score<min_score_ || visibility.rays<120 ||
      visibility.conflict_ratio>maximum_wall_conflict_) {return false;}
    double runner_score=-1.0;
    bool has_runner=false;
    for(size_t i=1;i<candidates.size();++i) {
      const auto & c=candidates[i];
      if(std::hypot(c.cell_x-best.cell_x,c.cell_y-best.cell_y)*map_.info.resolution<0.50 &&
        angular_distance(c.yaw_in_map,best.yaw_in_map)<15.0*kPi/180.0) {continue;}
      has_runner=true;
      const auto other=balanced_quality(c,sectors);
      runner_score=std::max(runner_score,other.score);
      // Additional evidence may validate the original winner, NEVER replace it.
      if(b.score-other.score<supplemental_margin_) {
        balanced_margin_=b.score-runner_score;
        return false;
      }
      if(c.score>=best.score-min_score_margin_) {
        const auto competing=visibility_quality(c,rays);
        if(competing.rays<120 || visibility.conflict_ratio>competing.conflict_ratio+0.02) {
          return false;
        }
      }
    }
    balanced_margin_=b.score-runner_score;
    return has_runner;
  }

  VisibilityQuality visibility_quality(const Candidate & c,
    const std::vector<ScanPoint> & points) const
  {
    VisibilityQuality result;
    std::array<bool,8> conflict_sectors{};
    const int w=map_.info.width, h=map_.info.height;
    const double res=map_.info.resolution;
    const double cosine=std::cos(c.yaw_in_map), sine=std::sin(c.yaw_in_map);
    for (const auto & p: points) {
      if (!p.origin_valid) {continue;}
      const double vx=p.x-p.origin_x, vy=p.y-p.origin_y;
      const double length=std::hypot(vx,vy);
      if (length<0.75) {continue;}
      ++result.rays;
      const double ox=c.cell_x+(cosine*p.origin_x-sine*p.origin_y)/res;
      const double oy=c.cell_y+(sine*p.origin_x+cosine*p.origin_y)/res;
      const double dx=(cosine*vx-sine*vy)/length, dy=(sine*vx+cosine*vy)/length;
      // Ignore the emitter neighbourhood, endpoint uncertainty and grazing rays.
      for (double r=0.30; r<length-0.30; r+=res*0.5) {
        const int x=std::lround(ox+dx*r/res), y=std::lround(oy+dy*r/res);
        if (x<0 || y<0 || x>=w || y>=h) {continue;}
        const int index=y*w+x;
        const double tangent=map_tangent_[index];
        if (map_.data[index]<occupied_threshold_ || !std::isfinite(tangent) ||
          std::abs(-std::sin(tangent)*dx+std::cos(tangent)*dy)<0.30) {continue;}
        // Require at least 60 cm of continuous mapped line support, not chair legs.
        bool supported=true;
        for (double offset : {-0.30,-0.15,0.15,0.30}) {
          const int tx=std::lround(x+std::cos(tangent)*offset/res);
          const int ty=std::lround(y+std::sin(tangent)*offset/res);
          if (tx<0 || ty<0 || tx>=w || ty>=h || map_.data[ty*w+tx]<0 ||
            distance_cells_[ty*w+tx]*res>0.10) {supported=false; break;}
        }
        if (supported) {
          ++result.conflicts;
          conflict_sectors[std::clamp(static_cast<int>((std::atan2(vy,vx)+kPi)*4/kPi),0,7)]=true;
          break;
        }
      }
    }
    if(result.rays>0) result.conflict_ratio=static_cast<double>(result.conflicts)/result.rays;
    result.conflict_sectors=std::count(conflict_sectors.begin(),conflict_sectors.end(),true);
    return result;
  }

  bool valid_robot_cell(double cell_x, double cell_y) const
  {
    const int x = static_cast<int>(std::lround(cell_x));
    const int y = static_cast<int>(std::lround(cell_y));
    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    if (x < 0 || y < 0 || x >= width || y >= height) {
      return false;
    }
    const std::size_t index = static_cast<std::size_t>(y * width + x);
    const int occupancy = map_.data[index];
    return occupancy >= 0 && occupancy <= free_threshold_ &&
           distance_cells_[index] * map_.info.resolution >= robot_clearance_;
  }

  bool match(
    const std::vector<ScanPoint> & scan_points, Candidate & best, Candidate & second)
  {
    last_quality_=MatchQuality{};
    close_points_.clear();
    std::vector<ScanPoint> normal_points;
    for(const auto & p:scan_points) {
      if(p.near_return) {if(near_wall_matching_ && p.origin_valid) close_points_.push_back(p);}
      else normal_points.push_back(p);
    }
    // Every retained return already passed a five-beam continuity check.
    // Do not drop the constraint simply because a near-range patch has <20 beams.
    if(close_points_.size()<8) close_points_.clear();
    near_mode_valid_=true;
    close_points_=uniformly_sample(close_points_,24);
    supplemental_unique_=false; balanced_margin_=0.0; wall_conflict_=0.0;
    // A short near-range patch supplements, never replaces, scene geometry.
    if (normal_points.size()<static_cast<size_t>(min_scan_points_)) {return false;}
    if (occupied_cell_count_ == 0) {
      RCLCPP_ERROR(get_logger(), "Map contains no occupied cells");
      return false;
    }

    const auto coarse_points = uniformly_sample(normal_points, coarse_scan_points_);
    const auto fine_points = uniformly_sample(normal_points, fine_scan_points_);
    if (diagnostic_reference_enabled_) {
      const double origin_cos = std::cos(map_origin_yaw_);
      const double origin_sin = std::sin(map_origin_yaw_);
      const double delta_x = diagnostic_reference_x_ - map_.info.origin.position.x;
      const double delta_y = diagnostic_reference_y_ - map_.info.origin.position.y;
      const double local_x = origin_cos * delta_x + origin_sin * delta_y;
      const double local_y = -origin_sin * delta_x + origin_cos * delta_y;
      const double cell_x = local_x / map_.info.resolution - 0.5;
      const double cell_y = local_y / map_.info.resolution - 0.5;
      const double yaw_in_map = normalize_angle(
        diagnostic_reference_yaw_ - map_origin_yaw_);
      const auto reference_offsets = rotate_scan(fine_points, yaw_in_map);
      RCLCPP_INFO(
        get_logger(),
        "Diagnostic reference score at [%.3f, %.3f, %.3f]: %.4f (valid_cell=%s)",
        diagnostic_reference_x_, diagnostic_reference_y_, diagnostic_reference_yaw_,
        score_candidate(cell_x, cell_y, reference_offsets),
        valid_robot_cell(cell_x, cell_y) ? "true" : "false");
    }
    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    const int cell_stride = std::max(
      1, static_cast<int>(std::lround(coarse_linear_step_ / map_.info.resolution)));
    const double coarse_angle_step = coarse_angular_step_deg_ * kPi / 180.0;
    const int angle_count = std::max(
      1, static_cast<int>(std::ceil(2.0 * kPi / coarse_angle_step)));

    // Keep a regional champion per yaw bin BEFORE the global top-K cut.
    // Otherwise all 40 entries can be adjacent samples of the same corridor.
    std::map<std::array<int,3>, Candidate> regional;
    const double region_cells=candidate_region_m_/map_.info.resolution;
    for (int angle_index = 0; angle_index < angle_count; ++angle_index) {
      const double yaw = -kPi + angle_index * (2.0 * kPi / angle_count);
      const auto offsets = rotate_scan(coarse_points, yaw);
      for (int y = cell_stride / 2; y < height; y += cell_stride) {
        for (int x = cell_stride / 2; x < width; x += cell_stride) {
          if (!valid_robot_cell(x, y)) {
            continue;
          }
          Candidate candidate;
          candidate.cell_x = x;
          candidate.cell_y = y;
          candidate.yaw_in_map = yaw;
          candidate.score = score_candidate(x, y, offsets);
          if(!close_points_.empty()) candidate.score*=0.65+0.35*close_score(x,y,yaw,0.20);
          const std::array<int,3> key{static_cast<int>(x/region_cells),
            static_cast<int>(y/region_cells), angle_index*12/angle_count};
          auto found=regional.find(key);
          if (found==regional.end() || candidate.score>found->second.score) {
            regional[key]=candidate;
          }
        }
      }
    }

    if (regional.empty()) {
      return false;
    }
    std::vector<Candidate> coarse_candidates;
    for (const auto & entry : regional) {coarse_candidates.push_back(entry.second);}
    std::sort(
      coarse_candidates.begin(), coarse_candidates.end(),
      [](const Candidate & lhs, const Candidate & rhs) {return lhs.score > rhs.score;});

    std::vector<Candidate> seeds;
    std::map<std::array<int,2>, int> region_count;
    for (const auto & candidate : coarse_candidates) {
      const std::array<int,2> key{static_cast<int>(candidate.cell_x/region_cells),
        static_cast<int>(candidate.cell_y/region_cells)};
      if (region_count[key]>=2) {continue;}
      bool duplicate = false;
      for (const auto & existing : seeds) {
        const double dx =
          (candidate.cell_x - existing.cell_x) * map_.info.resolution;
        const double dy =
          (candidate.cell_y - existing.cell_y) * map_.info.resolution;
        if (
          std::hypot(dx, dy) < candidate_region_m_ * 0.70 &&
          angular_distance(candidate.yaw_in_map, existing.yaw_in_map) <
          20.0*kPi/180.0)
        {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        seeds.push_back(candidate);
        ++region_count[key];
        if (static_cast<int>(seeds.size())>=top_candidates_) {break;}
      }
    }

    std::vector<Candidate> refined;
    const double fine_angle_window = fine_angular_window_deg_ * kPi / 180.0;
    const double fine_angle_step = fine_angular_step_deg_ * kPi / 180.0;
    const double fine_cell_step = fine_linear_step_ / map_.info.resolution;
    const double fine_cell_window = fine_linear_window_ / map_.info.resolution;
    for (const auto & seed : seeds) {
      Candidate seed_best = seed;
      // Scores from the 90-point coarse set MUST NOT survive into the fine
      // stage: every seed, neighbour, finalist and runner-up uses these same points.
      seed_best.score=ranking_score(seed.cell_x, seed.cell_y,seed.yaw_in_map,
        rotate_scan(fine_points,seed.yaw_in_map));
      for (
        double angle_delta = -fine_angle_window;
        angle_delta <= fine_angle_window + 1e-9;
        angle_delta += fine_angle_step)
      {
        const double yaw = normalize_angle(seed.yaw_in_map + angle_delta);
        const auto offsets = rotate_scan(fine_points, yaw);
        for (
          double delta_y = -fine_cell_window;
          delta_y <= fine_cell_window + 1e-9;
          delta_y += fine_cell_step)
        {
          for (
            double delta_x = -fine_cell_window;
            delta_x <= fine_cell_window + 1e-9;
            delta_x += fine_cell_step)
          {
            const double cell_x = seed.cell_x + delta_x;
            const double cell_y = seed.cell_y + delta_y;
            if (!valid_robot_cell(cell_x, cell_y)) {
              continue;
            }
            const double score = ranking_score(cell_x, cell_y,yaw, offsets);
            if (score > seed_best.score) {
              seed_best = {cell_x, cell_y, yaw, score};
            }
          }
        }
      }
      refined.push_back(continuous_refine(seed_best, fine_points));
    }

    std::sort(
      refined.begin(), refined.end(),
      [](const Candidate & lhs, const Candidate & rhs) {return lhs.score > rhs.score;});
    if (refined.empty()) {
      return false;
    }
    best = refined.front();
#ifdef RELOCALIZATION_DIAGNOSTIC
    diagnostic_candidates_ = refined;
#endif
    last_quality_=evaluate_candidate(best.cell_x,best.cell_y,rotate_scan(fine_points,best.yaw_in_map));
    if(!close_points_.empty()) {
      const auto visibility=visibility_quality(best,uniformly_sample(normal_points,480));
      near_mode_valid_=close_score(best.cell_x,best.cell_y,best.yaw_in_map)>=0.65 &&
        visibility.rays>=120 && visibility.conflict_ratio<=maximum_wall_conflict_;
      RCLCPP_INFO(get_logger(),"Near-wall verification: points=%zu fit=%.3f wall_conflict=%.3f valid=%s",
        close_points_.size(),close_score(best.cell_x,best.cell_y,best.yaw_in_map),visibility.conflict_ratio,
        near_mode_valid_?"true":"false");
    }
    RCLCPP_INFO(get_logger(),
      "Robust verification: matched=%.3f structure=%.3f sectors=%d directions=%d normal_diversity=%.3f seeds=%zu",
      last_quality_.matched_ratio,last_quality_.structural_ratio,
      last_quality_.sectors,last_quality_.orientations,last_quality_.normal_diversity,seeds.size());
    second = Candidate{};
    for (std::size_t index = 1; index < refined.size(); ++index) {
      const double dx = (refined[index].cell_x - best.cell_x) * map_.info.resolution;
      const double dy = (refined[index].cell_y - best.cell_y) * map_.info.resolution;
      if (
        std::hypot(dx, dy) >= 0.50 ||
        angular_distance(refined[index].yaw_in_map, best.yaw_in_map) >= 15.0 * kPi / 180.0)
      {
        second = refined[index];
        break;
      }
    }
    if(supplemental_verification_ && second.score>0.0 && best.score>=min_score_ &&
      best.score-second.score<min_score_margin_) {
      supplemental_unique_=verify_ambiguous(scan_points,refined);
      const auto bp=candidate_to_pose(best), sp=candidate_to_pose(second);
      RCLCPP_INFO(get_logger(),
        "Ambiguity verification: best=[%.3f,%.3f,%.3f] second=[%.3f,%.3f,%.3f] "
        "balanced_margin=%.4f wall_conflict=%.3f supplemental_unique=%s",
        bp[0],bp[1],bp[2],sp[0],sp[1],sp[2],balanced_margin_,wall_conflict_,
        supplemental_unique_?"true":"false");
    }
    return true;
  }

  Candidate continuous_refine(Candidate best, const std::vector<ScanPoint> & points) const
  {
    // Bounded derivative-free optimization on a bilinear distance field.
    // Coordinates are doubles, not quantized to the 5 cm correlation grid.
    const Candidate anchor=best;
    double translation=0.025/map_.info.resolution, rotation=0.5*kPi/180;
    for (int iteration=0; iteration<24; ++iteration) {
      bool improved=false;
      for (int axis=0; axis<3; ++axis) {
        for (int sign : {-1,1}) {
          Candidate c=best;
          if (axis==0) {c.cell_x+=sign*translation;}
          if (axis==1) {c.cell_y+=sign*translation;}
          if (axis==2) {c.yaw_in_map=normalize_angle(c.yaw_in_map+sign*rotation);}
          if (!valid_robot_cell(c.cell_x,c.cell_y) ||
            std::hypot(c.cell_x-anchor.cell_x,c.cell_y-anchor.cell_y)*map_.info.resolution>0.10 ||
            angular_distance(c.yaw_in_map,anchor.yaw_in_map)>2*kPi/180) {continue;}
          c.score=ranking_score(c.cell_x,c.cell_y,c.yaw_in_map,rotate_scan(points,c.yaw_in_map));
          if (c.score>best.score+1e-7) {best=c; improved=true;}
        }
      }
      if (!improved) {translation*=0.5; rotation*=0.5;}
      if (translation*map_.info.resolution<0.002 && rotation<0.05*kPi/180) {break;}
    }
    return best;
  }

  std::array<double, 3> candidate_to_pose(const Candidate & candidate) const
  {
    const double local_x = (candidate.cell_x + 0.5) * map_.info.resolution;
    const double local_y = (candidate.cell_y + 0.5) * map_.info.resolution;
    const double origin_cos = std::cos(map_origin_yaw_);
    const double origin_sin = std::sin(map_origin_yaw_);
    return {
      map_.info.origin.position.x + origin_cos * local_x - origin_sin * local_y,
      map_.info.origin.position.y + origin_sin * local_x + origin_cos * local_y,
      normalize_angle(map_origin_yaw_ + candidate.yaw_in_map)
    };
  }

  void publish_result(double x, double y, double yaw, double score)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = map_frame_;
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.position.z = 0.0;
    pose.pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.pose.orientation.w = std::cos(yaw * 0.5);
    pose.pose.covariance.fill(0.0);
    pose.pose.covariance[0] = covariance_x_;
    pose.pose.covariance[7] = covariance_y_;
    pose.pose.covariance[35] = covariance_yaw_;

    pose.header.stamp = now();
    result_pose_pub_->publish(pose);
    if (!publish_initial_pose_) {
      RCLCPP_INFO(
        get_logger(), "Dry run: result pose published only (score %.4f)", score);
      return;
    }

    for (int index = 0; index < std::max(1, publish_repetitions_); ++index) {
      pose.header.stamp = now();
      initialpose_pub_->publish(pose);
      if (index + 1 < publish_repetitions_) {
        rclcpp::sleep_for(std::chrono::milliseconds(150));
      }
    }
    RCLCPP_INFO(
      get_logger(), "Published global pose to %s %d time(s)",
      initialpose_topic_.c_str(), std::max(1, publish_repetitions_));
  }

  void retry_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    enabled_ = true;
    completed_ = false;
    supplemental_unique_=false; balanced_margin_=0.0; wall_conflict_=0.0;
    attempt_count_ = 0;
    have_consistency_candidate_ = false;
    consistent_match_count_ = 0;
    last_attempt_time_ = {};
    last_front_stamp_=last_rear_stamp_=0;
    response->success = true;
    response->message = "global relocalization re-armed";
    publish_status("waiting", "manual retry requested");
  }

  void publish_status(
    const std::string & state, const std::string & detail,
    double score = 0.0, double elapsed = 0.0)
  {
    if (!status_pub_) {
      return;
    }
    std_msgs::msg::String status;
    std::ostringstream stream;
    stream << "{\"state\":\"" << state << "\",\"detail\":\"" << detail
           << "\",\"attempt\":" << attempt_count_
           << ",\"score\":" << score
           << ",\"elapsed_sec\":" << elapsed
           << ",\"matched_ratio\":" << last_quality_.matched_ratio
           << ",\"structural_ratio\":" << last_quality_.structural_ratio
           << ",\"matched_sectors\":" << last_quality_.sectors
           << ",\"normal_diversity\":" << last_quality_.normal_diversity
           << ",\"supplemental_unique\":" << (supplemental_unique_?"true":"false")
           << ",\"balanced_margin\":" << balanced_margin_
           << ",\"wall_conflict\":" << wall_conflict_ << "}";
    status.data = stream.str();
    status_pub_->publish(status);
  }

  std::string map_topic_;
  std::string scan_topic_;
  std::string rear_scan_topic_;
  std::string initialpose_topic_;
  std::string result_pose_topic_;
  std::string status_topic_;
  std::string base_frame_;
  std::string map_frame_;

  bool auto_start_{true};
  bool publish_initial_pose_{true};
  bool use_rear_scan_{true};
  bool require_rear_scan_{true};
  bool enabled_{true};
  bool completed_{false};
  bool matching_{false};
  bool map_ready_{false};
  int occupied_threshold_{65};
  int free_threshold_{25};
  int min_scan_points_{60};
  int coarse_scan_points_{90};
  int fine_scan_points_{240};
  int top_candidates_{40};
  int max_attempts_{8};
  int publish_repetitions_{3};
  int required_consistent_matches_{3};
  int attempt_count_{0};
  int consistent_match_count_{0};
  double min_range_{0.2};
  double max_range_{15.0};
  double coarse_linear_step_{0.25};
  double coarse_angular_step_deg_{5.0};
  double fine_linear_window_{0.35};
  double fine_linear_step_{0.05};
  double fine_angular_window_deg_{6.0};
  double fine_angular_step_deg_{1.0};
  double likelihood_sigma_{0.12};
  double max_obstacle_distance_{0.75};
  double robot_clearance_{0.20};
  double min_score_{0.52};
  double min_score_margin_{0.03};
  double scan_pair_max_age_sec_{0.30};
  double scan_max_age_sec_{0.60};
  double minimum_matched_ratio_{0.35}, minimum_structural_ratio_{0.15};
  double minimum_normal_diversity_{0.06};
  double occluded_point_weight_{0.45}, candidate_region_m_{1.0};
  int minimum_matched_sectors_{3};
  std::string odom_frame_{"odom"};
  MatchQuality last_quality_;
  std::vector<int> nearest_occupied_;
  std::vector<float> map_structure_, map_tangent_;
  double consistency_position_tolerance_{0.25};
  double consistency_yaw_tolerance_deg_{5.0};
  double retry_interval_sec_{1.0};
  double transform_timeout_sec_{0.3};
  double covariance_x_{0.25};
  double covariance_y_{0.25};
  double covariance_yaw_{0.20};
  bool diagnostic_reference_enabled_{false};
  double diagnostic_reference_x_{0.0};
  double diagnostic_reference_y_{0.0};
  double diagnostic_reference_yaw_{0.0};
  double map_origin_yaw_{0.0};
  std::size_t occupied_cell_count_{0};

  nav_msgs::msg::OccupancyGrid map_;
  std::vector<float> distance_cells_;
  std::vector<float> likelihood_lut_;
  std::deque<sensor_msgs::msg::LaserScan::SharedPtr> front_scans_, rear_scans_;
  std::mutex scan_mutex_;
  int64_t last_front_stamp_{0}, last_rear_stamp_{0};
  rclcpp::CallbackGroup::SharedPtr scan_group_;
  rclcpp::TimerBase::SharedPtr search_timer_;
  std::mutex data_mutex_;
  std::chrono::steady_clock::time_point last_attempt_time_;
  bool have_consistency_candidate_{false};
  Candidate consistency_candidate_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr result_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr retry_service_;
  bool supplemental_verification_{true}, supplemental_unique_{false};
  std::vector<ScanPoint> close_points_;
  bool near_wall_matching_{true}, near_mode_valid_{true};
  double supplemental_margin_{0.06}, maximum_wall_conflict_{0.15};
  double balanced_margin_{0.0}, wall_conflict_{0.0};
#ifdef RELOCALIZATION_DIAGNOSTIC
  std::vector<Candidate> diagnostic_candidates_;
#endif
};

#ifndef RELOCALIZATION_TEST
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  // The executor stores weak node references. Keep ownership until spin ends.
  auto node = std::make_shared<CartographerGlobalRelocalization>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
#endif
