#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

# Nodo que simula un sensor de distancia publicando datos falsos
class SensorSim(Node):
    def __init__(self):
        super().__init__('sensor_sim')
        self.publisher_ = self.create_publisher(Float32, '/sensor/distance', 10)
        self.timer = self.create_timer(0.05, self.publish_fake_data)
        self.counter = 0.0
        self.get_logger().info('MODO SIMULACIÓN: Generando datos falsos...')

    # Función para publicar datos falsos, hace que se mueva el cubito adelante y atrás
    def publish_fake_data(self):
        distance_m = 2.0 + 1.5 * math.sin(self.counter)
        msg = Float32()
        msg.data = distance_m
        self.publisher_.publish(msg)
        self.counter += 0.09

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SensorSim())
    rclpy.shutdown()

if __name__ == '__main__':
    main()