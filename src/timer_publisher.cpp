#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"  // Include if using standard messages
// Include custom message header if used
// #include "timer_publisher_cpp/msg/my_message.hpp"

class TimerPublisher : public rclcpp::Node
{
public:
  TimerPublisher()
    : Node("timer_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);  // Adjust topic name and queue size

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&TimerPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimerPublisher>());
  rclcpp::shutdown();
  return 0;
}
