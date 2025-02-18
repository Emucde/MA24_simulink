#include <casadi_mpc_publisher/casadi_mpc_publisher.hpp>

CasadiMPCPublisher::CasadiMPCPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<mpc_interfaces::msg::ControlArray>("u_input", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&CasadiMPCPublisher::timer_callback, this));
    }

void CasadiMPCPublisher::timer_callback()
    {
      auto message = mpc_interfaces::msg::ControlArray();
      Eigen::Map<Eigen::VectorXd> u_k(message.control_array.data(), 6);
      u_k = Eigen::VectorXd::Constant(6, 0.1);
      publisher_->publish(message);
    }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CasadiMPCPublisher>());
  rclcpp::shutdown();
  return 0;
}
