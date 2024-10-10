#include "ranging_along_edges.hpp"

Ranging ::Ranging(const std::string& name) : nav2_util::LifecycleNode(name) {}
// 析构函数的实现
Ranging::~Ranging() {
    // 清理
}

void Ranging::ranging_along_edges(const sensor_msgs::msg::LaserScan::ConstSharedPtr laser_msg) {
    sensor_msgs::msg::LaserScan scan = *laser_msg;
    float angle_install = ANGLE_TO_RADIAN(49.85);
    int index_zero = (int)((0 - scan.angle_min) / scan.angle_increment);
    float zero_length = scan.ranges[index_zero];
    float crossbeam_b = zero_length * sin(angle_install);  //横梁b的长度，小车90度垂直墙面的距离。
    RCLCPP_INFO(rclcpp::get_logger("ranging_edges"), "crossbeam_b:%f", crossbeam_b);
    //横梁a的计算，取302.756195度的laserscan计算横梁a的长度。
    int index_beam = (int)((ANGLE_TO_RADIAN(302.756195) - scan.angle_min) / scan.angle_increment);
    float beam_length = scan.ranges[index_beam];
    float height = sqrt(pow(zero_length, 2) - pow(crossbeam_b, 2));
    float crossbeam_a = sqrt(pow(beam_length, 2) - pow(height, 2));
    RCLCPP_INFO(rclcpp::get_logger("ranging_edges"), "crossbeam_a:%f", crossbeam_a);
    //通过公式计算小车离墙面的偏离角度。
    float angle_D = ANGLE_TO_RADIAN(360 - 302.756195);
    float D = sqrt(pow(beam_length, 2) + pow(zero_length, 2) - 2 * beam_length * zero_length * cos(angle_D));
    float angle_the = acos((pow(crossbeam_a, 2) + pow(crossbeam_b, 2) - pow(D, 2)) / 2 * crossbeam_a *
                           crossbeam_b);  //横梁a和横梁b的夹角。
    float angle_alpha =
        atan((crossbeam_a * cos(angle_the) - crossbeam_b) / (crossbeam_a * sin(angle_the)));  //墙面偏离角度计算
    RCLCPP_INFO(rclcpp::get_logger("ranging_edges"), "angle_alpha : %f", angle_alpha);
    //距离墙面的距离
    float distance_wall = crossbeam_b * cos(angle_alpha);
    RCLCPP_INFO(rclcpp::get_logger("ranging_edges"), "distance_wall : %f", distance_wall);
    beefast_interfaces::msg::AlongEdges ranging_msg;
    ranging_msg.distance = distance_wall;
    ranging_msg.alpha = angle_alpha;
    pub_distance_->publish(ranging_msg);
}

nav2_util::CallbackReturn Ranging::on_configure(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(rclcpp::get_logger("ranging_edges"), "Configuring...");
    laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser", rclcpp::SensorDataQoS(), std::bind(&Ranging::ranging_along_edges, this, std::placeholders::_1));
    pub_distance_ = this->create_publisher<beefast_interfaces::msg::AlongEdges>("/ranging_along_edges", 1);

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Ranging::on_activate(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(rclcpp::get_logger("ranging_edges"), "Activating...");

    pub_distance_->on_activate();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Ranging::on_deactivate(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(rclcpp::get_logger("ranging_edges"), "Deactivating...");

    pub_distance_->on_deactivate();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Ranging::on_cleanup(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(rclcpp::get_logger("ranging_edges"), "Cleaning up...");

    pub_distance_.reset();
    laser_subscription_.reset();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Ranging::on_shutdown(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(rclcpp::get_logger("ranging_edges"), "Shutting down...");
    return nav2_util::CallbackReturn::SUCCESS;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto pub_distance_node = std::make_shared<Ranging>("ranging_along_edges");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(pub_distance_node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}