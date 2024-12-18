#include "thor_controller/thor_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace thor_controller
{

std::string compensateZeros(const int value)
{
  std::string compensate_zeros = "";
  if(value < 10){
    compensate_zeros = "00";
  } else if(value < 100){
    compensate_zeros = "0";
  } else {
    compensate_zeros = "";
  }
  return compensate_zeros;
}
  
ThorInterface::ThorInterface()
{
}


ThorInterface::~ThorInterface()
{
  if (thor_.IsOpen())
  {
    try
    {
      thor_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("ThorInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


CallbackReturn ThorInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("ThorInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  position_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  prev_position_commands_.reserve(info_.joints.size());
  RCLCPP_INFO_STREAM(rclcpp::get_logger("ThorInterface"), "ON INIT - Joints Size: " << info_.joints.size());

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> ThorInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> ThorInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn ThorInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("ThorInterface"), "Starting robot hardware ...");

  // Reset commands and states
  position_commands_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  prev_position_commands_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  position_states_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  try
  {
    thor_.Open(port_);
    thor_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("ThorInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("ThorInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn ThorInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("ThorInterface"), "Stopping robot hardware ...");

  if (thor_.IsOpen())
  {
    try
    {
      thor_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("ThorInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ThorInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type ThorInterface::read(const rclcpp::Time &time,
                                                          const rclcpp::Duration &period)
{
  // Open Loop Control - assuming the robot is always where we command to be
  position_states_ = position_commands_;

  // Leer datos enviados por Thor

  // if (thor_.IsOpen() && thor_.IsDataAvailable() > 0)
  // {
  //   std::string serial_data;
  //   try
  //   {
  //     thor_.ReadLine(serial_data);
  //     if (!serial_data.empty())
  //     {
  //       //auto msg = std_msgs::msg::String();
  //       //msg.data = serial_data;
  //       //thor_output_publisher_->publish(msg);
  //       RCLCPP_INFO_STREAM(rclcpp::get_logger("ThorInterface"), "Received from Thor: " << serial_data);
  //     }
  //   }
  //   catch (...)
  //   {
  //     RCLCPP_ERROR(rclcpp::get_logger("ThorInterface"), "Failed to read data from Thor.");
  //   }
  // }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ThorInterface::write(const rclcpp::Time &time,
                                                           const rclcpp::Duration &period)
{
  if (position_commands_ == prev_position_commands_)
  {
    // Nothing changed, do not send any command
    return hardware_interface::return_type::OK;
  }

  // RCLCPP_INFO_STREAM(rclcpp::get_logger("ThorInterface"), "  1:" << position_commands_.at(0) << "  2:" << position_commands_.at(1) << "  3:" << position_commands_.at(2) << "  4:" << position_commands_.at(3) << "  5:" << position_commands_.at(4) << "  6:" << position_commands_.at(5));

  std::string msg;
  msg.append("G0 ");
  int art1 = 90 - static_cast<int>(((position_commands_.at(0) + (M_PI / 2)) * 180) / M_PI);
  msg.append("X");
  //msg.append(compensateZeros(art1));
  msg.append(std::to_string(art1));
  msg.append(" ");
  int art2 = 90 - static_cast<int>(((position_commands_.at(1) + (M_PI / 2)) * 180) / M_PI);
  msg.append("Y");
  // msg.append(compensateZeros(art2));
  msg.append(std::to_string(art2));
  msg.append(" ");
  int art3 = 90 - static_cast<int>(((position_commands_.at(2) + (M_PI / 2)) * 180) / M_PI);
  msg.append("Z");
  // msg.append(compensateZeros(art3));
  msg.append(std::to_string(art3));
  msg.append(" ");
  int art4 = static_cast<int>(((-position_commands_.at(3)) * 180) / (M_PI / 2));
  msg.append("U");
  // msg.append(compensateZeros(art4));
  msg.append(std::to_string(art4));
  msg.append(" ");
  int art5 = static_cast<int>(((-position_commands_.at(4)) * 180) / (M_PI / 2));
  msg.append("V");
  // msg.append(compensateZeros(art5));
  msg.append(std::to_string(art5));
  msg.append(" ");
  int art6 = static_cast<int>(((-position_commands_.at(5)) * 180) / (M_PI / 2));
  msg.append("W");
  // msg.append(compensateZeros(art6));
  msg.append(std::to_string(art6));
  msg.append("\r\n");

  try
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("ThorInterface"), "Sending new command " << msg);
    thor_.Write(msg);
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ThorInterface"),
                        "Something went wrong while sending the message "
                            << msg << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  prev_position_commands_ = position_commands_;

  return hardware_interface::return_type::OK;
}
}

PLUGINLIB_EXPORT_CLASS(thor_controller::ThorInterface, hardware_interface::SystemInterface)
