#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/wireless_controller.hpp>

class FwdMotionNode : public rclcpp::Node {
 public:
  FwdMotionNode() : Node("fwd_motion") {
    publisher_ = this->create_publisher<unitree_go::msg::WirelessController>(
        "/wirelesscontroller", 10);
    RCLCPP_INFO(get_logger(), "Ready to publish to /wirelesscontroller");

    subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", 10,
        std::bind(&FwdMotionNode::cmd_vel_callback, this,
                  std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to messages from /cmd_vel");
  }

 private:
  void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    auto out_msg = unitree_go::msg::WirelessController();
    out_msg.ly = msg->twist.linear.x;
    out_msg.rx = -msg->twist.angular.z;
    publisher_->publish(out_msg);
  }

  rclcpp::Publisher<unitree_go::msg::WirelessController>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FwdMotionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
