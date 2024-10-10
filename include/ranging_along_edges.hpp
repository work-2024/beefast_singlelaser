#pragma once
#include "lifecycle_single_laser.hpp"
#include "beefast_interfaces/msg/along_edges.hpp"


#define ANGLE_TO_RADIAN(angle) ((angle)*3141.5926/180000) // degress unit transform to radin
#define RADIAN_TO_ANGLE(angle) ((angle)*180000/3141.5926) // radin uint transform to degress 

class Ranging : public nav2_util::LifecycleNode
{
    public:
        Ranging(const std::string &name);
        ~Ranging();

        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    private:
        void ranging_along_edges(const sensor_msgs::msg::LaserScan::ConstSharedPtr laser_msg);

        rclcpp_lifecycle::LifecyclePublisher<beefast_interfaces::msg::AlongEdges>::SharedPtr pub_distance_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;



};