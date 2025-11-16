#include <rclcpp/rclcpp.hpp>
#include <ur5_teleop_msgs/msg/pose_delta.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <memory>

class KeyboardTeleopNode : public rclcpp::Node
{
public:
  KeyboardTeleopNode() : Node("ur5_keyboard_teleop")
  {
    publisher_ = this->create_publisher<ur5_teleop_msgs::msg::PoseDelta>("/ur5/teleop_delta", 10);
    
    // Declare parameters
    this->declare_parameter("linear_step", 0.01);   // 1cm per keypress
    this->declare_parameter("angular_step", 0.05);  // ~3 degrees per keypress
    
    linear_step_ = this->get_parameter("linear_step").as_double();
    angular_step_ = this->get_parameter("angular_step").as_double();
    
    RCLCPP_INFO(this->get_logger(), "UR5 Keyboard Teleop Started");
    RCLCPP_INFO(this->get_logger(), "Linear step: %.3f m, Angular step: %.3f rad", linear_step_, angular_step_);
    printInstructions();
  }

  void run()
  {
    // Set terminal to raw mode
    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    char c;
    while (rclcpp::ok())
    {
      if (read(STDIN_FILENO, &c, 1) > 0)
      {
        auto msg = ur5_teleop_msgs::msg::PoseDelta();
        bool publish = true;

        switch(c)
        {
          // Translation in X (forward/backward)
          case 'w': msg.dx = linear_step_; break;
          case 's': msg.dx = -linear_step_; break;
          
          // Translation in Y (left/right)
          case 'a': msg.dy = linear_step_; break;
          case 'd': msg.dy = -linear_step_; break;
          
          // Translation in Z (up/down)
          case 'q': msg.dz = linear_step_; break;
          case 'e': msg.dz = -linear_step_; break;
          
          // Rotation: Roll
          case 'r': msg.droll = angular_step_; break;
          case 'f': msg.droll = -angular_step_; break;
          
          // Rotation: Pitch
          case 't': msg.dpitch = angular_step_; break;
          case 'g': msg.dpitch = -angular_step_; break;
          
          // Rotation: Yaw
          case 'y': msg.dyaw = angular_step_; break;
          case 'h': msg.dyaw = -angular_step_; break;
          
          // Help
          case 'i':
            printInstructions();
            publish = false;
            break;
          
          // Quit
          case 'x':
          case 27: // ESC
            RCLCPP_INFO(this->get_logger(), "Shutting down...");
            tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
            rclcpp::shutdown();
            return;
          
          default:
            publish = false;
            break;
        }

        if (publish)
        {
          publisher_->publish(msg);
          RCLCPP_DEBUG(this->get_logger(), "Published: dx=%.3f, dy=%.3f, dz=%.3f, dr=%.3f, dp=%.3f, dyw=%.3f",
                      msg.dx, msg.dy, msg.dz, msg.droll, msg.dpitch, msg.dyaw);
        }
      }
      
      rclcpp::spin_some(this->shared_from_this());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Restore terminal
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
  }

private:
  void printInstructions()
  {
    std::cout << "\n=== UR5 Keyboard Teleoperation ===" << std::endl;
    std::cout << "Translation:" << std::endl;
    std::cout << "  W/S: +X/-X (forward/back)" << std::endl;
    std::cout << "  A/D: +Y/-Y (left/right)" << std::endl;
    std::cout << "  Q/E: +Z/-Z (up/down)" << std::endl;
    std::cout << "\nRotation:" << std::endl;
    std::cout << "  R/F: +Roll/-Roll" << std::endl;
    std::cout << "  T/G: +Pitch/-Pitch" << std::endl;
    std::cout << "  Y/H: +Yaw/-Yaw" << std::endl;
    std::cout << "\nCommands:" << std::endl;
    std::cout << "  I: Show instructions" << std::endl;
    std::cout << "  X or ESC: Quit" << std::endl;
    std::cout << "==================================\n" << std::endl;
  }

  rclcpp::Publisher<ur5_teleop_msgs::msg::PoseDelta>::SharedPtr publisher_;
  double linear_step_;
  double angular_step_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleopNode>();
  node->run();
  return 0;
}
