#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>
#include <rmw/qos_profiles.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class LidarFusionNode : public rclcpp::Node
{
public:
    using LaserScan = sensor_msgs::msg::LaserScan;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<LaserScan, LaserScan>;

    LidarFusionNode()
    : Node("lidar_fusion_node")
    {
        RCLCPP_INFO(this->get_logger(), "Lidar fusion node started");

        this->declare_parameter("front_topic", "/topic_gv_front_lidar_0_307");
        this->declare_parameter("rear_topic", "/topic_gv_rear_lidar_0_307");
        this->declare_parameter("output_topic", "/fusioned_cloud");
        this->declare_parameter("output_frame", "base_link");
        this->declare_parameter("range_min", 0.1);
        this->declare_parameter("range_max", 30.0);
        this->declare_parameter("voxel_size", 0.05);
        this->declare_parameter("sync_queue_size", 20);
        this->declare_parameter("front_translation", std::vector<double>{0.3, 0.0, 0.2});
        this->declare_parameter("front_rotation_rpy", std::vector<double>{M_PI, 0.0, 0.0});
        this->declare_parameter("rear_translation", std::vector<double>{-0.3, 0.0, 0.2});
        this->declare_parameter("rear_rotation_rpy", std::vector<double>{M_PI, 0.0, M_PI});

        front_topic_ = this->get_parameter("front_topic").as_string();
        rear_topic_ = this->get_parameter("rear_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        output_frame_ = this->get_parameter("output_frame").as_string();
        range_min_ = this->get_parameter("range_min").as_double();
        range_max_ = this->get_parameter("range_max").as_double();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        sync_queue_size_ = this->get_parameter("sync_queue_size").as_int();

        front_transform_ = buildTransform("front_translation", "front_rotation_rpy");
        rear_transform_ = buildTransform("rear_translation", "rear_rotation_rpy");

        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

        sub_front_.subscribe(this, front_topic_, rmw_qos_profile_sensor_data);
        sub_rear_.subscribe(this, rear_topic_, rmw_qos_profile_sensor_data);

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(sync_queue_size_), sub_front_, sub_rear_);
        sync_->registerCallback(
            std::bind(&LidarFusionNode::syncedCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(
            this->get_logger(),
            "Front lidar pose xyz=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
            front_transform_.translation().x(), front_transform_.translation().y(),
            front_transform_.translation().z(), front_rpy_.x(), front_rpy_.y(), front_rpy_.z());
        RCLCPP_INFO(
            this->get_logger(),
            "Rear lidar pose xyz=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
            rear_transform_.translation().x(), rear_transform_.translation().y(),
            rear_transform_.translation().z(), rear_rpy_.x(), rear_rpy_.y(), rear_rpy_.z());
    }

private:
    Eigen::Affine3f buildTransform(const std::string & translation_param, const std::string & rotation_param)
    {
        const auto translation_values = this->get_parameter(translation_param).as_double_array();
        const auto rotation_values = this->get_parameter(rotation_param).as_double_array();

        if (translation_values.size() != 3 || rotation_values.size() != 3) {
            throw std::runtime_error("Transform parameters must contain exactly 3 values");
        }

        const Eigen::Vector3f translation(
            static_cast<float>(translation_values[0]),
            static_cast<float>(translation_values[1]),
            static_cast<float>(translation_values[2]));
        const Eigen::Vector3f rpy(
            static_cast<float>(rotation_values[0]),
            static_cast<float>(rotation_values[1]),
            static_cast<float>(rotation_values[2]));

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() = translation;
        transform.linear() =
            (Eigen::AngleAxisf(rpy.z(), Eigen::Vector3f::UnitZ()) *
             Eigen::AngleAxisf(rpy.y(), Eigen::Vector3f::UnitY()) *
             Eigen::AngleAxisf(rpy.x(), Eigen::Vector3f::UnitX()))
                .toRotationMatrix();

        if (translation_param.find("front") != std::string::npos) {
            front_rpy_ = rpy;
        } else {
            rear_rpy_ = rpy;
        }

        return transform;
    }

    void syncedCallback(
        const LaserScan::ConstSharedPtr & front_msg,
        const LaserScan::ConstSharedPtr & rear_msg)
    {
        auto front_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        auto rear_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        auto merged_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        scanToCloud(front_msg, front_cloud);
        scanToCloud(rear_msg, rear_cloud);

        pcl::transformPointCloud(*front_cloud, *front_cloud, front_transform_);
        pcl::transformPointCloud(*rear_cloud, *rear_cloud, rear_transform_);

        *merged_cloud = *front_cloud + *rear_cloud;
        if (merged_cloud->empty()) {
            return;
        }

        if (voxel_size_ > 0.0) {
            pcl::VoxelGrid<pcl::PointXYZ> filter;
            filter.setInputCloud(merged_cloud);
            filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            filter.filter(*filtered);
            merged_cloud = filtered;
        }

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*merged_cloud, cloud_msg);
        cloud_msg.header.stamp = makeMonotonicStamp(
            newerStamp(front_msg->header.stamp, rear_msg->header.stamp));
        cloud_msg.header.frame_id = output_frame_;
        pub_cloud_->publish(cloud_msg);
    }

    void scanToCloud(
        const LaserScan::ConstSharedPtr & scan,
        pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const
    {
        cloud->clear();
        cloud->reserve(scan->ranges.size());

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            const float range = scan->ranges[i];
            if (!std::isfinite(range) || range < range_min_ || range > range_max_) {
                continue;
            }

            const float angle = scan->angle_min + static_cast<float>(i) * scan->angle_increment;
            pcl::PointXYZ pt;
            pt.x = range * std::cos(angle);
            pt.y = range * std::sin(angle);
            pt.z = 0.0f;
            cloud->push_back(pt);
        }
    }

    builtin_interfaces::msg::Time newerStamp(
        const builtin_interfaces::msg::Time & first,
        const builtin_interfaces::msg::Time & second) const
    {
        if (first.sec > second.sec || (first.sec == second.sec && first.nanosec >= second.nanosec)) {
            return first;
        }
        return second;
    }

    builtin_interfaces::msg::Time makeMonotonicStamp(const builtin_interfaces::msg::Time & stamp)
    {
        if (!has_last_cloud_stamp_) {
            last_cloud_stamp_ = stamp;
            has_last_cloud_stamp_ = true;
            return stamp;
        }

        if (stamp.sec > last_cloud_stamp_.sec ||
            (stamp.sec == last_cloud_stamp_.sec && stamp.nanosec > last_cloud_stamp_.nanosec)) {
            last_cloud_stamp_ = stamp;
            return stamp;
        }

        builtin_interfaces::msg::Time adjusted = last_cloud_stamp_;
        if (adjusted.nanosec == 999999999u) {
            adjusted.nanosec = 0u;
            adjusted.sec += 1;
        } else {
            adjusted.nanosec += 1u;
        }

        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Received out-of-order fused cloud stamp, adjusted to keep points2 monotonic");

        last_cloud_stamp_ = adjusted;
        return adjusted;
    }

    message_filters::Subscriber<LaserScan> sub_front_;
    message_filters::Subscriber<LaserScan> sub_rear_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;

    std::string front_topic_;
    std::string rear_topic_;
    std::string output_topic_;
    std::string output_frame_;

    Eigen::Affine3f front_transform_;
    Eigen::Affine3f rear_transform_;
    Eigen::Vector3f front_rpy_{Eigen::Vector3f::Zero()};
    Eigen::Vector3f rear_rpy_{Eigen::Vector3f::Zero()};
    builtin_interfaces::msg::Time last_cloud_stamp_{};
    bool has_last_cloud_stamp_{false};

    double range_min_{0.1};
    double range_max_{30.0};
    double voxel_size_{0.05};
    int sync_queue_size_{20};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
