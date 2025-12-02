// src/mecanum_controller.cpp
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace yahboomcar_mecanum_controller
{

class MecanumController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    try {
      // 声明参数（必须在 on_init 中）
      auto_declare<std::vector<std::string>>("wheels", std::vector<std::string>());
      auto_declare<double>("wheel_radius", 0.0325);
      auto_declare<double>("wheel_separation_width", 0.169);
      auto_declare<double>("wheel_separation_length", 0.16);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = wheel_names_;
    return config;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::return_type update(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) override
  {
    if (!cmd_vel_) {
      return controller_interface::return_type::OK;
    }

    double vx = cmd_vel_->linear.x;
    double vy = cmd_vel_->linear.y;
    double wz = cmd_vel_->angular.z;

    // 从节点获取参数（注意：必须通过 get_node()）
    double r = get_node()->get_parameter("wheel_radius").as_double();
    double L = get_node()->get_parameter("wheel_separation_length").as_double() / 2.0;
    double W = get_node()->get_parameter("wheel_separation_width").as_double() / 2.0;

    // 麦克纳姆轮逆运动学（FL, FR, RL, RR）
    double v_fl = (vx - vy - (L + W) * wz) / r;
    double v_fr = (vx + vy + (L + W) * wz) / r;
    double v_rl = (vx + vy - (L + W) * wz) / r;
    double v_rr = (vx - vy + (L + W) * wz) / r;

    if (command_interfaces_.size() == 4) {
      command_interfaces_[0].set_value(v_fl);
      command_interfaces_[1].set_value(v_fr);
      command_interfaces_[2].set_value(v_rl);
      command_interfaces_[3].set_value(v_rr);
    }

    return controller_interface::return_type::OK;
  }

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    // 获取参数
    wheel_names_ = get_node()->get_parameter("wheels").as_string_array();
    if (wheel_names_.size() != 4) {
      RCLCPP_ERROR(get_node()->get_logger(), "Exactly 4 wheels required! Got %zu", wheel_names_.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    // 订阅 cmd_vel
    cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel",  // 注意：会自动加 controller 名前缀，如 /mecanum_controller/cmd_vel
      rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_vel_ = msg;
      });

    RCLCPP_INFO(get_node()->get_logger(), "MecanumController configured with wheels: %s, %s, %s, %s",
                wheel_names_[0].c_str(), wheel_names_[1].c_str(),
                wheel_names_[2].c_str(), wheel_names_[3].c_str());
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    // 初始化命令值为 0
    for (auto & interface : command_interfaces_) {
      interface.set_value(0.0);
    }
    cmd_vel_.reset();
    RCLCPP_INFO(get_node()->get_logger(), "MecanumController activated.");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    cmd_vel_.reset();
    for (auto & interface : command_interfaces_) {
      interface.set_value(0.0);
    }
    RCLCPP_INFO(get_node()->get_logger(), "MecanumController deactivated.");
    return controller_interface::CallbackReturn::SUCCESS;
  }

private:
  std::vector<std::string> wheel_names_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel_;
};

}  // namespace yahboomcar_mecanum_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  yahboomcar_mecanum_controller::MecanumController,
  controller_interface::ControllerInterface)