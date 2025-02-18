#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mpc_interfaces/msg/num.hpp"
#include "mpc_interfaces/msg/control_array.hpp"
#include <Eigen/Dense>

// #define MY_LOG_LEVEL  RCUTILS_LOG_SEVERITY_WARN
#define MY_LOG_LEVEL RCUTILS_LOG_SEVERITY_INFO

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class CasadiMPCPublisher : public rclcpp::Node
{
  public:
    CasadiMPCPublisher();

  private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<mpc_interfaces::msg::ControlArray>::SharedPtr publisher_;
    size_t count_;
};