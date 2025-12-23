#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class SimpleController : public rclcpp::Node
{
public:
  SimpleController()
  : Node("simple_controller")
  {
    // Publisher: thrust command [T_L, T_R]
    thrust_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/cmd_thrust", 10);

    // Subscriber: boat state [x, y, psi, u, v, r]
    state_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/boat_state", 10,
      std::bind(&SimpleController::state_callback, this, std::placeholders::_1));

    // Control loop timer
    timer_ = this->create_wall_timer(50ms,
      std::bind(&SimpleController::control_loop, this));

    start_time_ = now();
    RCLCPP_INFO(this->get_logger(), "Boat controller started");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr thrust_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<float> state_{0,0,0,0,0,0};  // x,y,psi,u,v,r

  rclcpp::Time start_time_;

  void state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 6)
      state_ = msg->data;
  }

  void control_loop()
  {
    double t = (now() - start_time_).seconds();

    // Initializing left and right trust
    float T_L = 0.0f;
    float T_R = 0.0f;

    // Simple control logic:
    //  - Drive forward for 20 seconds
    //  - Then turn right by increasing starboard thrust

    if (t < 20.0) {
      T_L = 20.0f;
      T_R = 20.0f;
    } else {
      T_L = 15.0f;
      T_R = 25.0f;
    }

    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {T_L, T_R};

    thrust_pub_->publish(msg);

    // RCLCPP_INFO_STREAM(this->get_logger(),
    //    "t=" << t << "  T_L=" << T_L << "  T_R=" << T_R
    //          << "  psi=" << state_[2] << "  u=" << state_[3]);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleController>());
  rclcpp::shutdown();
  return 0;
}
