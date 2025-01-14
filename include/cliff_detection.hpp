#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <tuple>
#include <vector>

#include "beefast_interfaces/msg/cliff_detection.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "ros2_api.h"

using CallbackReturn = nav2_util::CallbackReturn;

class CliffDetectionNode : public nav2_util::LifecycleNode {
  public:
    CliffDetectionNode(const std::string& name) : nav2_util::LifecycleNode(name) {}
    virtual ~CliffDetectionNode() {}

  protected:
    // 生命周期节点的回调函数
    CallbackReturn on_configure(const rclcpp_lifecycle::State&);

    CallbackReturn on_activate(const rclcpp_lifecycle::State&);

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&);

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&);

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&);

  private:
    // 激光雷达数据的回调函数，用于检测悬崖
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr);
    void SetLimitDistance(float);
    std::tuple<int, int, int> MaxAdjacentEqualCount(const std::vector<float>&);
    rclcpp_lifecycle::LifecyclePublisher<beefast_interfaces::msg::CliffDetection>::SharedPtr PubCliflag_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr LaserSubscription_;
    std::shared_ptr<rclcpp::ParameterEventHandler> ParamSubscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> ParamHandle_;
    float LimitDistance_;
    std::mutex LimitDistanceMtx_;
};
