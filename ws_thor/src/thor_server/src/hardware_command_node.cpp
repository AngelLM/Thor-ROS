#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <fstream> // Para manejar archivos

class HardwareCommandNode : public rclcpp::Node
{
public:
  HardwareCommandNode() : Node("hardware_command_node")
  {
    using std::placeholders::_1;
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/hardware_command", 10,
      std::bind(&HardwareCommandNode::command_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Nodo de comandos de hardware iniciado.");
  }

private:
  void command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Recibido comando: '%s'", msg->data.c_str());

    // Abrir el archivo en modo de agregar (append)
    std::ofstream file("/tmp/commands.txt", std::ios::app);
    if (file.is_open()) {
      file << msg->data << "\n"; // Escribir el comando en una nueva línea
      file.close();
      RCLCPP_INFO(this->get_logger(), "Comando guardado en /tmp/commands.txt.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "No se pudo abrir /tmp/commands.txt para escribir.");
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareCommandNode>());
  rclcpp::shutdown();
  return 0;
}