#include <unitree/robot/go2/video/video_client.hpp>
#include <zmq.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

int main() {
    const std::string iface = "eth0";
    const std::string zmq_endpoint = "tcp://*:5555";

    std::cout << "Initializing Unitree ChannelFactory" << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(0, iface.c_str());

    std::cout << "Initializing VideoClient" << std::endl;
    unitree::robot::go2::VideoClient video;
    video.SetTimeout(1.0f);
    video.Init();

    std::cout << "Initializing ZeroMQ PUB socket" << std::endl;
    zmq::context_t ctx{1};
    zmq::socket_t pub{ctx, zmq::socket_type::pub};
    pub.bind(zmq_endpoint);

    std::cout << "Streaming video frames on " << zmq_endpoint << std::endl;

    bool savedImage = true;

    while (true) {
        std::vector<uint8_t> jpeg_data;
        int ret = video.GetImageSample(jpeg_data);

        if (ret == 0 && !jpeg_data.empty()) {
            // Decode JPEG to cv::Mat
            cv::Mat img = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
            if (img.empty()) {
                std::cerr << "Failed to decode JPEG frame" << std::endl;
                continue;
            }

            if (!savedImage) {
                savedImage = true;
                std::string image_name("test_front_camera.jpg");
                std::ofstream image_file(image_name, std::ios::binary);
                    if (image_file.is_open()) {
                    image_file.write(reinterpret_cast<const char*>(jpeg_data.data()), jpeg_data.size());
                    image_file.close();
                    std::cout << "Image saved successfully as " << image_name << std::endl;
                } else {
                    std::cerr << "Error: Failed to save image." << std::endl;
                }
            }

            // Downscale to 480p
            cv::Mat img_480p;
            cv::resize(img, img_480p, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);

            // Re-encode to JPEG
            std::vector<uint8_t> jpeg_out;
            if (!cv::imencode(".jpg", img_480p, jpeg_out)) {
                std::cerr << "Failed to re-encode JPEG" << std::endl;
                continue;
            }

            // Send over ZeroMQ
            zmq::message_t msg(jpeg_out.size());
            memcpy(msg.data(), jpeg_out.data(), jpeg_out.size());
            pub.send(msg, zmq::send_flags::none);
        } else {
            std::cerr << "No frame received" << std::endl;
        }

        // 5 fps -> 100ms
        std::this_thread::sleep_for(200ms);
    }

    return 0;
}
