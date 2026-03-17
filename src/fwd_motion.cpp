#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unitree_api/msg/request.hpp>

#include "common/ros2_sport_client.h"  // from third_party/unitree_ros2/example/src/include

class FwdMotionNode : public rclcpp::Node {
 public:
  FwdMotionNode() : Node("fwd_motion"), sport_client_(this) {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&FwdMotionNode::cmd_vel_callback, this,
                  std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "fwd_motion started, subscribed to /cmd_vel");
  }

 private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    unitree_api::msg::Request req;
    if (msg->linear.x == 0.0 && msg->linear.y == 0.0 && msg->angular.z == 0.0) {
      sport_client_.StopMove(req);
    } else {
      sport_client_.Move(req, msg->linear.x, msg->linear.y, msg->angular.z);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  SportClient sport_client_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FwdMotionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
