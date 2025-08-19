#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "pmw3901.hpp"
#include <chrono>

using namespace std::chrono_literals;

class PMW3901MotionNode : public rclcpp::Node {
  public:
    PMW3901MotionNode()
    : Node("pmw3901_motion")
    {
      pmw = std::make_shared<PMW3901>("/dev/spidev0.0", 8, 19); // CS=BCM8, MOTION=BCM19
      if (!pmw->begin()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize PMW3901 sensor");
        rclcpp::shutdown();
        return;
      }
      // QoS profile setting(KeepLast, best_effort)
      rclcpp::QoS qos_profile(rclcpp::KeepLast(5));
      qos_profile.best_effort();

      publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/pmw3901/motion", qos_profile);
      timer = this->create_wall_timer(20ms, std::bind(&PMW3901MotionNode::loop, this));
    }

  private:
    void loop() {
      int16_t dx, dy;
      pmw->read_motion_count(&dx, &dy);

      geometry_msgs::msg::Vector3Stamped msg;
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "pmw3901";
      msg.vector.x = -dx;
      msg.vector.y = dy;
      msg.vector.z = 0.0;
      publisher->publish(msg);
    }

    std::shared_ptr<PMW3901> pmw;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PMW3901MotionNode>());
  rclcpp::shutdown();
  return 0;
}
