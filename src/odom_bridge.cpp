#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <unitree_go/msg/sport_mode_state.hpp>

class OdomBridgeNode : public rclcpp::Node {
 public:
  OdomBridgeNode() : Node("odom_bridge") {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    subscription_ = this->create_subscription<unitree_go::msg::SportModeState>(
        "/sportmodestate", 10,
        std::bind(&OdomBridgeNode::state_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "odom_bridge started");
  }

 private:
  void state_callback(const unitree_go::msg::SportModeState::SharedPtr msg) {
    auto now = this->get_clock()->now();

    // Build quaternion from IMU RPY
    tf2::Quaternion q;
    q.setRPY(
      msg->imu_state.rpy[0],  // roll
      msg->imu_state.rpy[1],  // pitch
      msg->imu_state.rpy[2]   // yaw
    );

    // Publish odom → base_link TF
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = msg->position[0];
    tf.transform.translation.y = msg->position[1];
    tf.transform.translation.z = msg->position[2];
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);

    // Publish /odom
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = msg->position[0];
    odom.pose.pose.position.y = msg->position[1];
    odom.pose.pose.position.z = msg->position[2];
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = msg->velocity[0];
    odom.twist.twist.linear.y = msg->velocity[1];
    odom.twist.twist.linear.z = msg->velocity[2];
    odom.twist.twist.angular.z = msg->yaw_speed;
    odom_pub_->publish(odom);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
