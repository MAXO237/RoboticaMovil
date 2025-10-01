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
      float distance_cm = read_from_serial();
      
      if (distance_cm < 0.0f) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), 
          *this->get_clock(), 
          5000,
          "Invalid or no new data from sensor.");
        return;
      }

      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "/base_link";
      marker.header.stamp = this->get_clock()->now();

      marker.ns = "sensor_reading";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      
      marker.color.r = 0.1;
      marker.color.g = 0.5;
      marker.color.b = 1.0;
      marker.color.a = 0.8;

      marker.lifetime.sec = 2;

      marker.pose.position.x = distance_cm / 100.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.w = 1.0;

      this->_publisher->publish(marker);
    };
    
    _timer = this->create_wall_timer(100ms, timer_callback);
  }

  ~MarkerPublisher() {
    if (serial_port_ >= 0) {
      close(serial_port_);
    }
  }

private:
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _publisher;
  size_t _count;
  int serial_port_ = -1;

  void configure_serial_port() {
    serial_port_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);

    if (serial_port_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Could not open serial port '/dev/ttyACM0'. Check permissions and device connection.");
      return;
    }

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_port_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
        return;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Serial port '/dev/ttyACM0' configured successfully at 9600 baud.");
  }

  float read_from_serial() {
    if (serial_port_ < 0) return -1.0f;

    char read_buf[256];
    std::memset(&read_buf, '\0', sizeof(read_buf));
    
    int num_bytes = read(serial_port_, &read_buf, sizeof(read_buf) - 1);

    if (num_bytes > 0) {
      std::string data(read_buf);
      std::stringstream ss(data);
      float distance;
      ss >> distance;
      if (!ss.fail()) {
        return distance;
      }
    }
    
    return -1.0f;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}

