#pragma once
#include <chrono>
#include <memory>
#include <thread>

#include "nav2_util/lifecycle_node.hpp"
#include "ros2_api.h"
#include "rtrnet.h"


class SingleLaserLifecycle : public nav2_util::LifecycleNode {
  public:
    SingleLaserLifecycle(const std::string& name);
    ~SingleLaserLifecycle();

    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  private:
    void data_publish_loop();
    void ToLaserscanMessagePublish(ldlidar_ssl::Points2D& src);

    LaserScanSetting setting_;
    std::shared_ptr<ldlidar_ssl::RTRNet> lidar_comm_;
    std::shared_ptr<ldlidar_ssl::CmdInterfaceLinux> cmd_port_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_;
    std::thread data_thread_;
    bool stop_thread_ = false;
    int sleep_rate_ = 25;  // 默认设置
};
