#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "pmw3901.hpp"
#include <chrono>

using namespace std::chrono_literals;

class PMW3901FrameGrabNode : public rclcpp::Node {
  public:
    PMW3901FrameGrabNode()
    : Node("pmw3901_frame_grab")
    {
      pmw = std::make_shared<PMW3901>("/dev/spidev0.0", 8, 19);
      if (!pmw->begin()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize PMW3901 sensor");
        rclcpp::shutdown();
        return;
      }

      // QoS profile setting(KeepLast, best_effort)
      rclcpp::QoS qos_profile(rclcpp::KeepLast(5));
      qos_profile.best_effort();

      publisher = this->create_publisher<sensor_msgs::msg::Image>("/pmw3901/frame", qos_profile);
      timer = this->create_wall_timer(50ms, std::bind(&PMW3901FrameGrabNode::loop, this));
    }

  private:
    void loop() {
      std::vector<uint8_t> buffer;
      pmw->enable_frame_buffer();
      pmw->read_frame_buffer(buffer);

      sensor_msgs::msg::Image msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = "pmw3901";
      msg.height = 35;
      msg.width = 35;
      msg.encoding = "mono8";
      msg.step = msg.width;
      msg.is_bigendian = false;
      msg.data.assign(buffer.begin(), buffer.end());

      publisher->publish(msg);
    }

    std::shared_ptr<PMW3901> pmw;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PMW3901FrameGrabNode>());
  rclcpp::shutdown();
  return 0;
}
