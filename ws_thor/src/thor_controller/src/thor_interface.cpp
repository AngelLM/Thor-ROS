#include "thor_controller/thor_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace thor_controller
{

ThorInterface::ThorInterface()
{
}

ThorInterface::~ThorInterface()
{
  if(thor_.IsOpen()){
    try{
      thor_.Close();
    }
    catch(...){
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("ThorInterface"), "Something went wrong while closing connection with port " << port_);
    }
  }
}

CallbackReturn ThorInterface::on_init(const hardware_interface::HardwareInfo &hardware_info){
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if(result != CallbackReturn::SUCCESS){
    return result;
  }

  try{
    port_ = info_.hardware_parameters.at("port");
    board_type_ = info_.hardware_parameters.at("board_type");
  }
  catch(const std::out_of_range &e){
    RCLCPP_FATAL(rclcpp::get_logger("ThorInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  position_commands_.reserve(info_.joints.size());
  curr_angles_.reserve(info_.joints.size());
  prev_angles_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  RCLCPP_INFO_STREAM(rclcpp::get_logger("ThorInterface"), "ON INIT - Joints Size: " << info_.joints.size());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ThorInterface::export_state_interfaces(){
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide only a position interface
  for(size_t i = 0; i < info_.joints.size(); i++){
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ThorInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a position interface
  for(size_t i = 0; i < info_.joints.size(); i++){
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn ThorInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("ThorInterface"), "Starting robot hardware ...");

  // Reset commands and states
  position_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  curr_angles_ = {0, 0, 0, 0, 0, 0, 0};
  prev_angles_ = {0, 0, 0, 0, 0, 0, 0};
  position_states_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  try{
    thor_.Open(port_);
    thor_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    // Wait for a couple of seconds to let the board initialize
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  catch(...){
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("ThorInterface"), "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("ThorInterface"), "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ThorInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state){
  RCLCPP_INFO(rclcpp::get_logger("ThorInterface"), "Stopping robot hardware ...");

  if(thor_.IsOpen()){
    try{
      thor_.Close();
    }
    catch(...){
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("ThorInterface"), "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ThorInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

// Read data from Thor
hardware_interface::return_type ThorInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period){
  // If the port is open and no data is available, send a request for status
  if(thor_.IsOpen() && thor_.IsDataAvailable() <= 0){
    try{
      if (board_type_== "thor_pcb"){
        std::string msg = "?\r\n";
        thor_.Write(msg);
        // Wait for 0.05 seconds
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      else
        thor_.Write("M408\r\n");
    }
    catch(...){ 
      RCLCPP_ERROR(rclcpp::get_logger("ThorInterface"), "Failed to send a request for status to Thor.");
    }
  }

  if(thor_.IsOpen() && thor_.IsDataAvailable() > 0){
    std::string serial_data;
    try{
      thor_.ReadLine(serial_data);
      if(!serial_data.empty()){
        if(board_type_ == "thor_pcb"){
          // Check if the data contains the string "Mpos:"
          if(serial_data.find("MPos:") != std::string::npos){         
            // Extract the state of the board
            size_t state_end = serial_data.find(',');
            std::string board_state = serial_data.substr(1, state_end - 1);
            bool homed = (board_state == "Idle" || board_state == "Run");

            if (!homed) {
              RCLCPP_ERROR_STREAM(rclcpp::get_logger("ThorInterface"), "Board state is not homed: " << board_state);
            }

            // Extract the MPos values
            size_t mpos_start = serial_data.find("MPos:") + 5;
            size_t mpos_end = serial_data.find(",WPos:");
            std::string data = serial_data.substr(mpos_start, mpos_end - mpos_start);
            std::istringstream iss(data);
            std::string token;
            int i = 0;
            while (std::getline(iss, token, ',') && i < position_states_.size()) {
              if (i == 1) {
              // Skip the third token as it is a duplicate of the second
              std::getline(iss, token, ',');
              }
              position_states_[i] = std::stod(token);
              i++;
            }

            position_states_[0] = position_states_[0] * M_PI / 180;
            position_states_[1] = position_states_[1] * M_PI / -180;
            position_states_[2] = position_states_[2] * M_PI / -180;
            position_states_[3] = position_states_[3] * M_PI / 180;
            double a6pos = (position_states_[4] + position_states_[5]) / 4;
            double a5pos = (position_states_[5] - 2 * a6pos);
            position_states_[4] = a5pos * M_PI / 180;
            position_states_[5] = a6pos * M_PI / 180;

            position_states_[6] = (curr_angles_[6]-180) * M_PI / 180;
          }
        }
        else{
          // Check if the data starts with a JSON object
          if(serial_data.find("{\"status\":") == 0){
            try{
              auto json_data = json::parse(serial_data);

              // Extract the status data
              if(json_data.contains("status")){
                auto status = json_data["status"];
              }

              // Extract the position data
              if(json_data.contains("pos")){
                auto pos = json_data["pos"];
                for(size_t i = 0; i < pos.size() && i < position_states_.size(); ++i){
                  position_states_[i] = pos[i];
                }

                position_states_[0] = position_states_[0] * M_PI / 180;
                position_states_[1] = position_states_[1] * M_PI / -180;
                position_states_[2] = position_states_[2] * M_PI / -180;
                position_states_[3] = position_states_[3] * M_PI / 180;
                double a6pos = (position_states_[4] + position_states_[5]) / 4;
                double a5pos = (position_states_[5] - 2 * a6pos);
                position_states_[4] = a5pos * M_PI / 180;
                position_states_[5] = a6pos * M_PI / 180;

                position_states_[6] = (curr_angles_[6]-180) * M_PI / 180;
              }
              
              // Extract the axis homed data
              if(json_data.contains("homed")){
                auto axis_homed = json_data["homed"];
                bool homed = true;
                for(size_t i = 0; i < axis_homed.size() && i < axis_homed.size(); ++i){
                  if(axis_homed[i] == 0){
                    homed = false;
                    break;
                  }
                }
              }
            } 
            catch(const json::parse_error &e){
              RCLCPP_ERROR(rclcpp::get_logger("ThorInterface"), "Failed to parse JSON: %s", e.what());
            }
          }
        }
      }
    }
    catch(...){
      RCLCPP_ERROR(rclcpp::get_logger("ThorInterface"), "Failed to read data from Thor.");
    }
  }

  // Process external commands
  // 
  // NOTE: This is a temporary workaround to handle the issue with memory addressing of the queues.
  // It works for now, but a proper solution needs to be implemented in the future.

  {
    // Check if the file "/tmp/commands.txt" exists
    std::ifstream file("/tmp/commands.txt");
    if(file.is_open()){
      std::string command;
      std::vector<std::string> lines;
      std::string line;

      // Read all lines from the file
      while(std::getline(file, line)){
        lines.push_back(line);
      }
      file.close();

      // Delete the file
      if (std::remove("/tmp/commands.txt") == 0){
        RCLCPP_INFO(rclcpp::get_logger("ThorInterface"), "File /tmp/commands.txt deleted.");
      }
      else{
        RCLCPP_ERROR(rclcpp::get_logger("ThorInterface"), "Failed to delete file /tmp/commands.txt.");
      }

      if(!lines.empty()){
        for(size_t i = 0; i < lines.size(); ++i){
          RCLCPP_INFO(rclcpp::get_logger("ThorInterface"), "Sending external command: %s", lines[i].c_str());
          thor_.Write(lines[i] + "\r\n");
        }
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ThorInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period){
  // Calculate angles
  int art1 = -90 + static_cast<int>(((position_commands_.at(0) + (M_PI / 2)) * 180) / M_PI);
  int art2 = 90 - static_cast<int>(((position_commands_.at(1) + (M_PI / 2)) * 180) / M_PI);
  int art3 = 90 - static_cast<int>(((position_commands_.at(2) + (M_PI / 2)) * 180) / M_PI);
  int art4 = static_cast<int>(((position_commands_.at(3)) * 180) / M_PI);
  int art5 = static_cast<int>(((-position_commands_.at(4)) * 180) / M_PI);
  int art6 = static_cast<int>(((position_commands_.at(5)) * 180) / M_PI);

  int m_art5 = art5 + 2 * art6;
  int m_art6 = -1 * art5 + 2 * art6;

  int end_effector = static_cast<int>(180 + (position_commands_.at(6) * 180 / M_PI));

  curr_angles_ = {art1, art2, art3, art4, m_art5, m_art6, end_effector};

  if(curr_angles_ == prev_angles_){
    return hardware_interface::return_type::OK;
  }

  if(curr_angles_.at(0) != prev_angles_.at(0) || curr_angles_.at(1) != prev_angles_.at(1) || curr_angles_.at(2) != prev_angles_.at(2) || curr_angles_.at(3) != prev_angles_.at(3) || curr_angles_.at(4) != prev_angles_.at(4) || curr_angles_.at(5) != prev_angles_.at(5)){
    // Build the GCODE message for arm movement
    std::string msg;
    if(board_type_ == "thor_pcb"){
      msg.append("G0 ");
      msg.append("A").append(std::to_string(art1)).append(" ");
      msg.append("B").append(std::to_string(art2)).append(" ");
      msg.append("C").append(std::to_string(art2)).append(" ");
      msg.append("D").append(std::to_string(art3)).append(" ");
      msg.append("X").append(std::to_string(art4)).append(" ");
      msg.append("Y").append(std::to_string(m_art5)).append(" ");
      msg.append("Z").append(std::to_string(m_art6)).append("\r\n");
    }
    else{
      msg.append("G0 ");
      msg.append("X").append(std::to_string(art1)).append(" ");
      msg.append("Y").append(std::to_string(art2)).append(" ");
      msg.append("Z").append(std::to_string(art3)).append(" ");
      msg.append("U").append(std::to_string(art4)).append(" ");
      msg.append("V").append(std::to_string(m_art5)).append(" ");
      msg.append("W").append(std::to_string(m_art6)).append("\r\n");
    }

    // Send the command
    try{
      RCLCPP_INFO_STREAM(rclcpp::get_logger("ThorInterface"), "Sending new command " << msg);
      thor_.Write(msg);
    }
    catch(...){
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("ThorInterface"), "Something went wrong while sending the message " << msg << " to the port " << port_);
      return hardware_interface::return_type::ERROR;
    }
  }
  if(curr_angles_.at(6) != prev_angles_.at(6)){
    // Build the GCODE message for end effector movement
    std::string msg;
    if(board_type_ == "thor_pcb"){
      int pwm_value = static_cast<int>(end_effector * 255 / 180);
      msg.append("M3 ");
      msg.append("S").append(std::to_string(pwm_value)).append("\r\n");
    }
    else{
      msg.append("M280 P0 ");
      msg.append("S").append(std::to_string(end_effector)).append("\r\n");
    }

    // Send the command
    try{
      RCLCPP_INFO_STREAM(rclcpp::get_logger("ThorInterface"), "Sending new command " << msg);
      thor_.Write(msg);
    }
    catch(...){
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("ThorInterface"), "Something went wrong while sending the message " << msg << " to the port " << port_);
      return hardware_interface::return_type::ERROR;
    }

  }
    prev_angles_ = curr_angles_;

    return hardware_interface::return_type::OK;
  }
} // namespace thor_controller

PLUGINLIB_EXPORT_CLASS(thor_controller::ThorInterface, hardware_interface::SystemInterface)