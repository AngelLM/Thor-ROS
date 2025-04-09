#ifndef THOR_INTERFACE_H
#define THOR_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <vector>
#include <string>
#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>

namespace thor_controller
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using json = nlohmann::json;

class ThorInterface : public hardware_interface::SystemInterface
{
public:
  ThorInterface();
  virtual ~ThorInterface();

  // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // Implementing hardware_interface::SystemInterface
  virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;


private:
  LibSerial::SerialPort thor_;
  std::string port_, board_type_;
  std::vector<double> position_commands_;
  std::vector<int> curr_angles_;
  std::vector<int> prev_angles_;
  std::vector<double> position_states_;

  bool homed_;
  std::string status_;
};
} // namespace thor_controller


#endif  // THOR_INTERFACE_H