#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <vector>
#include <string>

class MyBotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  MyBotHardwareInterface() = default;
  virtual ~MyBotHardwareInterface() = default;

  // การตั้งค่าเริ่มต้น
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize joint states and commands
    joint_positions_.resize(info_.joints.size(), 0.0);
    joint_velocities_.resize(info_.joints.size(), 0.0);
    joint_position_commands_.resize(info_.joints.size(), 0.0);
    joint_velocity_commands_.resize(info_.joints.size(), 0.0);

    RCLCPP_INFO(rclcpp::get_logger("MyBotHardwareInterface"), "Successfully initialized MyBot Hardware Interface");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // กำหนด state interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
    }

    return state_interfaces;
  }

  // กำหนด command interfaces
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_commands_[i]));
    }

    return command_interfaces;
  }

  // เริ่มการทำงาน
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override
  {
    RCLCPP_INFO(rclcpp::get_logger("MyBotHardwareInterface"), "Starting MyBot Hardware Interface");
    
    // Reset all values
    for (size_t i = 0; i < joint_positions_.size(); ++i)
    {
      joint_positions_[i] = 0.0;
      joint_velocities_[i] = 0.0;
      joint_position_commands_[i] = 0.0;
      joint_velocity_commands_[i] = 0.0;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // หยุดการทำงาน
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override
  {
    RCLCPP_INFO(rclcpp::get_logger("MyBotHardwareInterface"), "Stopping MyBot Hardware Interface");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // อ่านข้อมูลจากฮาร์ดแวร์
  hardware_interface::return_type read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override
  {
    // ใน hardware จริง จะอ่านค่าจาก sensors/encoders
    // ตอนนี้จำลองโดยใช้ค่า command เป็น feedback
    for (size_t i = 0; i < joint_positions_.size(); ++i)
    {
      joint_positions_[i] = joint_position_commands_[i];
      joint_velocities_[i] = joint_velocity_commands_[i];
    }

    return hardware_interface::return_type::OK;
  }

  // ส่งคำสั่งไปยังฮาร์ดแวร์
  hardware_interface::return_type write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override
  {
    // ใน hardware จริง จะส่งคำสั่งไปยัง motors/actuators
    // ตอนนี้แค่ log ข้อมูล
    for (size_t i = 0; i < joint_position_commands_.size(); ++i)
    {
      // ส่งคำสั่งไปยัง motor ที่ joint i
      // เช่น motor_controller->setPosition(i, joint_position_commands_[i]);
    }

    return hardware_interface::return_type::OK;
  }

private:
  // Joint states
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  
  // Joint commands
  std::vector<double> joint_position_commands_;
  std::vector<double> joint_velocity_commands_;
};

// สำหรับ plugin loading
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MyBotHardwareInterface, hardware_interface::SystemInterface)

// Main function (ไม่จำเป็นสำหรับ hardware interface plugin)
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  RCLCPP_INFO(rclcpp::get_logger("my_bot"), "MyBot Hardware Interface Node Started");
  
  // Keep the node alive
  rclcpp::spin(std::make_shared<rclcpp::Node>("my_bot_node"));
  
  rclcpp::shutdown();
  return 0;
}
