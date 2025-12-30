#include <unitree/robot/go2/video/video_client.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <chrono>
#include <cstring>

#include <zenoh.h>

using namespace std::chrono_literals;

// CDR serialization helpers for ROS 2 CompressedImage message
// ROS 2 uses CDR (Common Data Representation) for message serialization

class CdrSerializer {
public:
    CdrSerializer() {
        buffer_.reserve(256 * 1024); // Reserve 256KB initially
    }

    void reset() {
        buffer_.clear();
        // CDR encapsulation header: 4 bytes
        // byte 0: 0x00 (big endian) or 0x01 (little endian)
        // byte 1: 0x00 (reserved)
        // bytes 2-3: 0x00 0x00 (options)
        buffer_.push_back(0x00); // Little endian
        buffer_.push_back(0x01); // CDR version
        buffer_.push_back(0x00); // Options
        buffer_.push_back(0x00); // Options
    }

    void serialize_uint32(uint32_t val) {
        align(4);
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&val);
        buffer_.insert(buffer_.end(), bytes, bytes + 4);
    }

    void serialize_int32(int32_t val) {
        align(4);
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&val);
        buffer_.insert(buffer_.end(), bytes, bytes + 4);
    }

    void serialize_string(const std::string& str) {
        // String length (including null terminator)
        serialize_uint32(static_cast<uint32_t>(str.size() + 1));
        // String data
        buffer_.insert(buffer_.end(), str.begin(), str.end());
        buffer_.push_back(0); // Null terminator
    }

    void serialize_byte_array(const uint8_t* data, size_t size) {
        // Array length
        serialize_uint32(static_cast<uint32_t>(size));
        // Array data (no alignment needed for bytes)
        buffer_.insert(buffer_.end(), data, data + size);
    }

    const std::vector<uint8_t>& data() const { return buffer_; }
    size_t size() const { return buffer_.size(); }

private:
    void align(size_t alignment) {
        size_t pos = buffer_.size();
        size_t padding = (alignment - (pos % alignment)) % alignment;
        for (size_t i = 0; i < padding; ++i) {
            buffer_.push_back(0);
        }
    }

    std::vector<uint8_t> buffer_;
};

// Serialize a sensor_msgs/msg/CompressedImage to CDR format
// Message definition:
//   std_msgs/Header header
//   string format
//   uint8[] data
std::vector<uint8_t> serialize_compressed_image_msg(
    const std::string& frame_id,
    int32_t sec,
    uint32_t nanosec,
    const std::string& format,
    const uint8_t* image_data,
    size_t data_size)
{
    CdrSerializer cdr;
    cdr.reset();

    // Header (std_msgs/msg/Header)
    //   stamp (builtin_interfaces/msg/Time)
    cdr.serialize_int32(sec);
    cdr.serialize_uint32(nanosec);
    //   frame_id (string)
    cdr.serialize_string(frame_id);

    // format (string)
    cdr.serialize_string(format);

    // data (sequence<uint8>)
    cdr.serialize_byte_array(image_data, data_size);

    return cdr.data();
}

int main() {
    const std::string iface = "eth0";
    const char* zenoh_topic = "camera/image_raw/compressed";

    std::cout << "Initializing Unitree ChannelFactory" << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(0, iface.c_str());

    std::cout << "Initializing VideoClient" << std::endl;
    unitree::robot::go2::VideoClient video;
    video.SetTimeout(1.0f);
    video.Init();

    std::cout << "Initializing Zenoh session" << std::endl;

    // Initialize zenoh config
    z_owned_config_t config;
    z_config_default(&config);

    // Open zenoh session
    z_owned_session_t session;
    if (z_open(&session, z_move(config), NULL) < 0) {
        std::cerr << "Failed to open Zenoh session" << std::endl;
        return 1;
    }

    // Declare publisher
    std::cout << "Creating Zenoh publisher on topic: " << zenoh_topic << std::endl;
    z_owned_publisher_t pub;
    z_view_keyexpr_t keyexpr;
    z_view_keyexpr_from_str(&keyexpr, zenoh_topic);

    if (z_declare_publisher(z_loan(session), &pub, z_loan(keyexpr), NULL) < 0) {
        std::cerr << "Failed to declare Zenoh publisher" << std::endl;
        z_drop(z_move(session));
        return 1;
    }

    std::cout << "Streaming compressed video frames on Zenoh topic: " << zenoh_topic << std::endl;

    while (true) {
        std::vector<uint8_t> jpeg_data;
        int ret = video.GetImageSample(jpeg_data);

        if (ret == 0 && !jpeg_data.empty()) {
            // Get current time for the message timestamp
            auto now = std::chrono::system_clock::now();
            auto duration = now.time_since_epoch();
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
            auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration) -
                               std::chrono::duration_cast<std::chrono::nanoseconds>(seconds);

            // Serialize the CompressedImage message to CDR format
            std::vector<uint8_t> cdr_data = serialize_compressed_image_msg(
                "camera_front",                           // frame_id
                static_cast<int32_t>(seconds.count()),    // sec
                static_cast<uint32_t>(nanoseconds.count()), // nanosec
                "jpeg",                                   // format
                jpeg_data.data(),                         // compressed image data
                jpeg_data.size()                          // data size
            );

            // Publish to Zenoh
            z_owned_bytes_t payload;
            z_bytes_copy_from_buf(&payload, cdr_data.data(), cdr_data.size());
            z_publisher_put(z_loan(pub), z_move(payload), NULL);
        } else {
            std::cerr << "No frame received" << std::endl;
        }

        // 30 fps -> 33ms
        std::this_thread::sleep_for(33ms);
    }

    // Cleanup (unreachable in this loop, but good practice)
    z_drop(z_move(pub));
    z_drop(z_move(session));

    return 0;
}
