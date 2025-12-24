#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <zmq.hpp>

class FwdCameraNode : public rclcpp::Node {
 public:
  FwdCameraNode()
      : Node("zmq_camera_node"), ctx_(1), sub_(ctx_, zmq::socket_type::sub), saved_image_(true) {
    publisher_ =
        create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);

    std::string endpoint = "tcp://127.0.0.1:5555";
    sub_.connect(endpoint);
    sub_.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&FwdCameraNode::poll, this));

    RCLCPP_INFO(get_logger(), "Subscribed to %s", endpoint.c_str());
  }

 private:
  void poll() {
    zmq::message_t msg;
    auto res = sub_.recv(msg, zmq::recv_flags::dontwait);

    if (!res) return;

    std::vector<uint8_t> jpeg(static_cast<uint8_t*>(msg.data()),
                              static_cast<uint8_t*>(msg.data()) + msg.size());
    
    cv::Mat img = cv::imdecode(jpeg, cv::IMREAD_COLOR);
    if (img.empty()) return;

    if (!saved_image_) {
      saved_image_ = true;
      std::vector<int> params = {
        cv::IMWRITE_JPEG_QUALITY, 90  // Range: 0–100
      };
      cv::imwrite("camera_after_cv.jpg", img, params);
    }

    auto ros_img =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();

    // ros_img->header.stamp = now();
    // ros_img->header.frame_id = "camera_front";

    publisher_->publish(*ros_img);
  }

  zmq::context_t ctx_;
  zmq::socket_t sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool saved_image_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FwdCameraNode>());
  rclcpp::shutdown();
  return 0;
}
