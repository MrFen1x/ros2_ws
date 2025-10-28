#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots/DistanceSensor.hpp>

#include "my_webots_robot/youbot_wrapper.hpp"

using namespace std::chrono_literals;

class MyRobotController : public webots_ros2_driver::WebotsNode {
public:
  explicit MyRobotController(const rclcpp::NodeOptions &options)
      : webots_ros2_driver::WebotsNode(options) {
    RCLCPP_INFO(get_logger(), "✅ My Webots robot node started");

    // Получаем указатель на устройство
    distance_sensor_ = getWebotsDevice<webots::DistanceSensor>("distance sensor");
    if (distance_sensor_) {
      distance_sensor_->enable(getBasicTimeStep());
      RCLCPP_INFO(get_logger(), "📡 Distance sensor enabled");
    } else {
      RCLCPP_ERROR(get_logger(), "❌ Distance sensor not found!");
    }

    base_init();
    arm_init();
    gripper_init();

    timer_ = create_wall_timer(1s, std::bind(&MyRobotController::automatic_behavior, this));
  }

private:
  webots::DistanceSensor *distance_sensor_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_ = 0;

  void automatic_behavior() {
    if (!distance_sensor_) return;

    double value_ds = distance_sensor_->getValue();
    RCLCPP_INFO(get_logger(), "Sensor reading: %.3f", value_ds);

    // Пример автоматического поведения
    gripper_release();
    arm_set_height(ARM_FRONT_CARDBOARD_BOX);
    arm_set_height(ARM_BACK_PLATE_LOW);
    gripper_grip();
    arm_reset();

    if (value_ds < 1000.0) count_++;
    RCLCPP_INFO(get_logger(), "✅ Перевёз %d кубиков", count_);
  }
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(MyRobotController)
