#include "lifecycle_single_laser.hpp"
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto ldlidar_node = std::make_shared<SingleLaserLifecycle>("ssl20_Sensor_Publisher");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ldlidar_node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}