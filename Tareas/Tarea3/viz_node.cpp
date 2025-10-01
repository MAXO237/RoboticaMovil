#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class MarkerPublisher : public rclcpp::Node
{
public:
  MarkerPublisher() : Node("marker_publisher"), _count(0)
  {
    _publisher = this->create_publisher<visualization_msgs::msg::Marker>("marker_topic", 10);
    
    configure_serial_port();

    auto timer_callback = 
    [this]() -> void {
      float distance_cm = read_from_se_

