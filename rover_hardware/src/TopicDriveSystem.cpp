#include "rover_hardware/TopicDriveSystem.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rover_hardware
{

hardware_interface::CallbackReturn TopicDriveSystemHardware::on_init(const hardware_interface::HardwareInfo & info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("TopicDriveSystem"));
    RCLCPP_INFO(*logger_, "Starting custom HURC hardware plugin!");

    joint_positions_.resize(info_.joints.size());
    joint_velocities_.resize(info_.joints.size());
    joint_commands_.resize(info_.joints.size());

    return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn TopicDriveSystemHardware::on_configure(const rclcpp_lifecycle::State&) {
    for (uint i = 0; i < joint_positions_.size(); i++) {
        joint_positions_[i] = 0;
        joint_velocities_[i] = 0;
        joint_commands_[i] = 0;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
};

std::vector<hardware_interface::StateInterface> TopicDriveSystemHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
    }
    return state_interfaces;
};

std::vector<hardware_interface::CommandInterface> TopicDriveSystemHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_commands_[i]));
    }
    return command_interfaces;
};

hardware_interface::CallbackReturn TopicDriveSystemHardware::on_activate(const rclcpp_lifecycle::State&) {
    return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn TopicDriveSystemHardware::on_deactivate(const rclcpp_lifecycle::State&) {
    return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::return_type TopicDriveSystemHardware::read(const rclcpp::Time&, const rclcpp::Duration& dt) {
    for (uint i = 0; i < joint_positions_.size(); i++) {
        joint_positions_[i] += joint_velocities_[i] * dt.seconds();
    }
    return hardware_interface::return_type::OK;
};

hardware_interface::return_type TopicDriveSystemHardware::write(const rclcpp::Time&, const rclcpp::Duration&) {
    for (uint i = 0; i < joint_positions_.size(); i++) {
        joint_velocities_[i] = joint_commands_[i];
    }
    return hardware_interface::return_type::OK;
};

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rover_hardware::TopicDriveSystemHardware, hardware_interface::SystemInterface)