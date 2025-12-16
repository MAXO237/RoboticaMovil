#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

class SensorReal(Node):
    def __init__(self):
        super().__init__('sensor_real')
        self.publisher_ = self.create_publisher(Float32, '/sensor/distance', 10)
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info('MODO REAL: Conectado a Arduino.')
            time.sleep(2)
        except Exception as e:
            self.get_logger().error(f'Error conectando Arduino: {e}')
            self.ser = None
        self.timer = self.create_timer(0.05, self.read_serial)

    def read_serial(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("Distance:"):
                    dist_m = float(line.replace("Distance:", "")) / 100.0
                    msg = Float32()
                    msg.data = dist_m
                    self.publisher_.publish(msg)
            except ValueError:
                pass

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SensorReal())
    rclpy.shutdown()

if __name__ == '__main__':
    main()