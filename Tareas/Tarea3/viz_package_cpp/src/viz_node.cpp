#include <cstdio>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/float32.hpp" // Para recibir los datos del sensor

using std::placeholders::_1;

class MarkerPublisher : public rclcpp::Node
{
  public:
    MarkerPublisher() : Node("marker_publisher")
    {
      // Se crea el publicador del marcador
      _marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("marker_topic", 10);
      
      // Se crea la suscripción al tópico del sensor
      // Cuando llegue un dato de distancia, se ejecuta 'topic_callback'
      _subscription = this->create_subscription<std_msgs::msg::Float32>(
      "/sensor/distance", 10, std::bind(&MarkerPublisher::topic_callback, this, _1));
      
      RCLCPP_INFO(this->get_logger(), "Nodo visualizador listo. Esperando datos en /sensor/distance...");
    }

  private:
    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) const
    {
        // Se crea el mensaje Marker
        auto marker = visualization_msgs::msg::Marker();
        
        // FRAME ID: Importante, relativo al robot 'base_link' [cite: 17]
        marker.header.frame_id = "base_link"; 
        marker.header.stamp = this->get_clock()->now();

        marker.type = visualization_msgs::msg::Marker::CUBE; 
        marker.id = 0;

        // Tamaño o escala del cubo (20 cm cada lado)
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        
        // Color verde y un poco transparente
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8; 

        // Vida del marcador
        marker.lifetime.sec = 1; 

        // Posición dinámica según el dato del sensor
        // msg->data viene en metros desde el nodo de python
        marker.pose.position.x = msg->data; 
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.1; 
        
        marker.pose.orientation.w = 1.0;

        // Publicar
        _marker_publisher->publish(marker);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _marker_publisher;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _subscription;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}