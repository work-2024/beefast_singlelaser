#include "cliff_detection.hpp"

#include <algorithm>
#include <mutex>

#include "ranging_along_edges.hpp"
constexpr int leftMinIndex = 79;
constexpr int rightMaxIndex = 421;

void CliffDetectionNode::SetLimitDistance(float limitDistance) {
    std::lock_guard<std::mutex> lock(LimitDistanceMtx_);
    LimitDistance_ = limitDistance;
}
std::tuple<int, int, int> CliffDetectionNode::MaxAdjacentEqualCount(const std::vector<float>& vec) {
    if (vec.empty()) {
        return { 0, -1, -1 };  // 如果向量为空，返回数量0，起始和结束位置为-1
    }

    int maxCount = 1;      // 最大相邻相同元素数量
    int currentCount = 1;  // 当前相邻相同元素数量
    int maxStart = 0;      // 最大相邻元素序列的起始位置
    int maxEnd = 0;        // 最大相邻元素序列的结束位置
    int currentStart = 0;  // 当前相邻相同元素序列的起始位置

    // 遍历向量，寻找相邻相同的元素
    for (size_t i = 0; i < vec.size(); ++i) {
        if (i > leftMinIndex && i <= rightMaxIndex)
            continue;
        if (std::isnan(vec[i]) || vec[i] > LimitDistance_) {
            ++currentCount;
        } else {
            // 检查是否需要更新最大值
            if (currentCount > maxCount) {
                maxCount = currentCount;
                maxStart = currentStart;
                maxEnd = i - 1;
            }
            // 重置计数器并更新当前序列的起始位置
            currentCount = 1;
            currentStart = i;
        }
    }

    // 最后再比较一次，因为可能最大序列在数组结尾
    if (currentCount > maxCount) {
        maxCount = currentCount;
        maxStart = currentStart;
        maxEnd = vec.size() - 1;
    }

    return { maxCount, maxStart, maxEnd };
}

CallbackReturn CliffDetectionNode::on_configure(const rclcpp_lifecycle::State& state) {
    this->declare_parameter<float>("LimitDistance", 0.5f);  //使用LimitDistance参数方便调试
    this->get_parameter("LimitDistance", LimitDistance_);

    LaserSubscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser", rclcpp::SensorDataQoS(),
        std::bind(&CliffDetectionNode ::laser_callback, this, std::placeholders::_1));
    PubCliflag_ = this->create_publisher<beefast_interfaces::msg::CliffDetection>("/cliffdetection", 1);

    ParamSubscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    // Lambda表达式
    auto fre_cb = [this](const rclcpp::Parameter& p) {
        //做自己的操作
        RCLCPP_INFO(this->get_logger(), "Parameter 'LimitDistance' changed to: %f", p.as_double());
        SetLimitDistance(p.as_double());
    };

    //注册参数变更回调
    ParamHandle_ = ParamSubscriber_->add_parameter_callback("LimitDistance", fre_cb);

    return CallbackReturn::SUCCESS;
}
CallbackReturn CliffDetectionNode::on_activate(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(rclcpp::get_logger("cliff_detection"), "activating...");

    PubCliflag_->on_activate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn CliffDetectionNode::on_deactivate(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(rclcpp::get_logger("cliff_detection"), "Deactivating...");
    PubCliflag_->on_deactivate();

    return CallbackReturn::SUCCESS;
}

CallbackReturn CliffDetectionNode::on_cleanup(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(rclcpp::get_logger("cliff_detection"), "Cleaning up...");
    LaserSubscription_.reset();
    PubCliflag_.reset();

    return CallbackReturn::SUCCESS;
}

CallbackReturn CliffDetectionNode::on_shutdown(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(rclcpp::get_logger("cliff_detection"), "Shutting down...");

    return CallbackReturn::SUCCESS;
}

void CliffDetectionNode::laser_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {  //得出激光距离并判断是否为楼梯
    sensor_msgs::msg::LaserScan scan = *msg;
    beefast_interfaces::msg::CliffDetection pubData;

    auto [counts, startPoint, endPoint] = MaxAdjacentEqualCount(scan.ranges);

    if (counts > 10) {
        pubData.cliflag = true;
        float startAngle = RADIAN_TO_ANGLE(startPoint * scan.angle_increment + scan.angle_min);
        float endAngle = RADIAN_TO_ANGLE(endPoint * scan.angle_increment + scan.angle_min);
        auto AngleProc = [](float* angle) -> void {
            if (*angle > 300.0f) {
                *angle -= 300.0f;
            }
        };
        AngleProc(&startAngle);
        AngleProc(&endAngle);
        float nowAngle = 0;
        if (startAngle >= 0 && endAngle >= 0) {
            nowAngle = (startAngle + endAngle) / 2;
        } else {
            nowAngle = startAngle + endAngle;
        }
        pubData.angle = nowAngle;
        goto end;
    }
    pubData.cliflag = false;
    pubData.angle = std::numeric_limits<float>::quiet_NaN();
end:
    PubCliflag_->publish(pubData);  //发布检测数据
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // auto pub_distance_node = std::make_shared< Ranging >( "ranging_along_edges" );
    auto pub_cliff_detection = std::make_shared<CliffDetectionNode>("cliff_detection");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(pub_cliff_detection->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}