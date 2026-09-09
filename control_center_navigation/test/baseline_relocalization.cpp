#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
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
};

struct RotatedOffset
{
  double x_cells{0.0};
  double y_cells{0.0};
};

struct Candidate
{
  double cell_x{0.0};
  double cell_y{0.0};
  double yaw_in_map{0.0};
  double score{0.0};
};

struct LowestScoreFirst
{
  bool operator()(const Candidate & lhs, const Candidate & rhs) const
  {
    return lhs.score > rhs.score;
  }
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
    min_score_margin_ = declare_parameter<double>("min_score_margin", 0.03);
    scan_pair_max_age_sec_ = declare_parameter<double>("scan_pair_max_age_sec", 0.30);
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

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&CartographerGlobalRelocalization::map_callback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_callback(msg, false);
      });
    if (use_rear_scan_) {
      rear_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        rear_scan_topic_,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          scan_callback(msg, true);
        });
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
    if (msg->info.width == 0 || msg->info.height == 0 || msg->info.resolution <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Rejected invalid occupancy grid");
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    map_ = *msg;
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
    occupied_cell_count_ = 0;

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const std::size_t index = static_cast<std::size_t>(y * width + x);
        if (map_.data[index] >= occupied_threshold_) {
          distance_cells_[index] = 0.0F;
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
        distance_cells_[index] = std::min(
          distance_cells_[index], distance_cells_[neighbor] + cost);
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
    if (!enabled_ || completed_ || matching_) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!map_ready_) {
        return;
      }
    }

    const auto now = std::chrono::steady_clock::now();
    if (
      last_attempt_time_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now - last_attempt_time_).count() < retry_interval_sec_)
    {
      return;
    }

    std::vector<ScanPoint> points;
    if (!laser_scan_in_base(*msg, points)) {
      return;
    }
    if (static_cast<int>(points.size()) < min_scan_points_) {
      RCLCPP_WARN(
        get_logger(), "%s laser has only %zu valid scan points; require at least %d",
        is_rear_scan ? "Rear" : "Front", points.size(), min_scan_points_);
      return;
    }

    const auto receipt_time = std::chrono::steady_clock::now();
    if (is_rear_scan) {
      rear_scan_points_ = std::move(points);
      rear_scan_time_ = receipt_time;
    } else {
      front_scan_points_ = std::move(points);
      front_scan_time_ = receipt_time;
    }

    std::vector<ScanPoint> combined_points;
    if (use_rear_scan_) {
      const bool have_front = !front_scan_points_.empty();
      const bool have_rear = !rear_scan_points_.empty();
      const double pair_age =
        (have_front && have_rear) ?
        std::abs(std::chrono::duration<double>(front_scan_time_ - rear_scan_time_).count()) :
        std::numeric_limits<double>::infinity();
      if (!have_front || !have_rear || pair_age > scan_pair_max_age_sec_) {
        if (require_rear_scan_ || !have_front) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Waiting for synchronized front/rear laser scans (front=%s rear=%s age=%.3fs)",
            have_front ? "ready" : "missing", have_rear ? "ready" : "missing", pair_age);
          publish_status("waiting_scans", "waiting for synchronized front/rear laser scans");
          return;
        }
        combined_points = front_scan_points_;
      } else {
        combined_points.reserve(front_scan_points_.size() + rear_scan_points_.size());
        combined_points.insert(
          combined_points.end(), front_scan_points_.begin(), front_scan_points_.end());
        combined_points.insert(
          combined_points.end(), rear_scan_points_.begin(), rear_scan_points_.end());
      }
    } else {
      combined_points = front_scan_points_;
    }

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
      second.score <= 0.0 || score_margin >= min_score_margin_;
    if (found && best.score >= min_score_ && unique_match) {
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
      publish_status("succeeded", "global pose published to AMCL", best.score, elapsed);
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "Global relocalization attempt %d rejected in %.3fs: best=%.4f second=%.4f "
      "margin=%.4f (required score>=%.4f margin>=%.4f)",
      attempt_count_, elapsed, best.score, second.score, score_margin,
      min_score_, min_score_margin_);
    publish_status(
      "retrying",
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
    std::vector<ScanPoint> & points)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        base_frame_, scan.header.frame_id, scan.header.stamp,
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
    for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
      const double range = scan.ranges[index];
      if (!std::isfinite(range) || range <= effective_min || range >= effective_max) {
        continue;
      }
      const double angle = scan.angle_min + index * scan.angle_increment;
      const double laser_x = range * std::cos(angle);
      const double laser_y = range * std::sin(angle);
      const tf2::Vector3 point_in_base =
        laser_to_base * tf2::Vector3(laser_x, laser_y, 0.0);
      points.push_back({point_in_base.x(), point_in_base.y()});
    }
    return true;
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
          (sine * point.x + cosine * point.y) * inverse_resolution
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

  void push_top_candidate(
    std::priority_queue<Candidate, std::vector<Candidate>, LowestScoreFirst> & heap,
    const Candidate & candidate) const
  {
    if (static_cast<int>(heap.size()) < top_candidates_) {
      heap.push(candidate);
      return;
    }
    if (candidate.score > heap.top().score) {
      heap.pop();
      heap.push(candidate);
    }
  }

  bool match(
    const std::vector<ScanPoint> & scan_points, Candidate & best, Candidate & second)
  {
    if (occupied_cell_count_ == 0) {
      RCLCPP_ERROR(get_logger(), "Map contains no occupied cells");
      return false;
    }

    const auto coarse_points = uniformly_sample(scan_points, coarse_scan_points_);
    const auto fine_points = uniformly_sample(scan_points, fine_scan_points_);
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

    std::priority_queue<Candidate, std::vector<Candidate>, LowestScoreFirst> coarse_heap;
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
          push_top_candidate(coarse_heap, candidate);
        }
      }
    }

    if (coarse_heap.empty()) {
      return false;
    }
    std::vector<Candidate> coarse_candidates;
    while (!coarse_heap.empty()) {
      coarse_candidates.push_back(coarse_heap.top());
      coarse_heap.pop();
    }
    std::sort(
      coarse_candidates.begin(), coarse_candidates.end(),
      [](const Candidate & lhs, const Candidate & rhs) {return lhs.score > rhs.score;});

    std::vector<Candidate> seeds;
    for (const auto & candidate : coarse_candidates) {
      bool duplicate = false;
      for (const auto & existing : seeds) {
        const double dx =
          (candidate.cell_x - existing.cell_x) * map_.info.resolution;
        const double dy =
          (candidate.cell_y - existing.cell_y) * map_.info.resolution;
        if (
          std::hypot(dx, dy) < coarse_linear_step_ * 0.75 &&
          angular_distance(candidate.yaw_in_map, existing.yaw_in_map) <
          coarse_angle_step * 0.75)
        {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        seeds.push_back(candidate);
      }
    }

    std::vector<Candidate> refined;
    const double fine_angle_window = fine_angular_window_deg_ * kPi / 180.0;
    const double fine_angle_step = fine_angular_step_deg_ * kPi / 180.0;
    const double fine_cell_step = fine_linear_step_ / map_.info.resolution;
    const double fine_cell_window = fine_linear_window_ / map_.info.resolution;
    for (const auto & seed : seeds) {
      Candidate seed_best = seed;
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
            const double score = score_candidate(cell_x, cell_y, offsets);
            if (score > seed_best.score) {
              seed_best = {cell_x, cell_y, yaw, score};
            }
          }
        }
      }
      refined.push_back(seed_best);
    }

    std::sort(
      refined.begin(), refined.end(),
      [](const Candidate & lhs, const Candidate & rhs) {return lhs.score > rhs.score;});
    if (refined.empty()) {
      return false;
    }
    best = refined.front();
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
    return true;
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
    attempt_count_ = 0;
    have_consistency_candidate_ = false;
    consistent_match_count_ = 0;
    last_attempt_time_ = {};
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
           << ",\"elapsed_sec\":" << elapsed << "}";
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
  std::vector<ScanPoint> front_scan_points_;
  std::vector<ScanPoint> rear_scan_points_;
  std::mutex data_mutex_;
  std::chrono::steady_clock::time_point last_attempt_time_;
  std::chrono::steady_clock::time_point front_scan_time_;
  std::chrono::steady_clock::time_point rear_scan_time_;
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
};

#ifndef RELOCALIZATION_TEST
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartographerGlobalRelocalization>());
  rclcpp::shutdown();
  return 0;
}
#endif
