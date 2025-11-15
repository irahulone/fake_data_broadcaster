import math
import random
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class WeatherSplitPublisher(Node):
    def __init__(self):
        super().__init__('weather_split_publisher')

        # === 4 Publishers (separate topics) ===
        self.pub_temp = self.create_publisher(Float32, 'temperature', 10)
        self.pub_humidity = self.create_publisher(Float32, 'humidity', 10)
        self.pub_wind_speed = self.create_publisher(Float32, 'wind_speed', 10)
        self.pub_wind_direction = self.create_publisher(Float32, 'wind_direction', 10)

        # Timer: 1 Hz
        self.timer = self.create_timer(1.0, self.timer_callback)

        # 96 slots = 24 hours simulated in 96 seconds
        self.num_slots = 96
        self.slot_idx = 0

        self.get_logger().info("WeatherSplitPublisher started.")

    def timer_callback(self):
        t_norm = self.slot_idx / float(self.num_slots)

        # Temperature (°C)
        temp = 20.0 + 8.0 * math.cos(2 * math.pi * (t_norm - 0.25)) + random.uniform(-0.5, 0.5)

        # Humidity (%)
        humidity = 60.0 - 15.0 * math.cos(2 * math.pi * (t_norm - 0.25)) + random.uniform(-2, 2)
        humidity = max(0.0, min(100.0, humidity))

        # Wind speed (m/s)
        wind_speed = 1.0 + 3.0 * max(0.0, math.sin(2 * math.pi * (t_norm - 0.25))) + random.uniform(-0.5, 0.5)
        wind_speed = max(0.0, wind_speed)

        # Wind direction (deg)
        wind_direction = (360.0 * t_norm + random.uniform(-20, 20)) % 360.0

        # Publish separate messages
        self.pub_temp.publish(Float32(data=float(temp)))
        self.pub_humidity.publish(Float32(data=float(humidity)))
        self.pub_wind_speed.publish(Float32(data=float(wind_speed)))
        self.pub_wind_direction.publish(Float32(data=float(wind_direction)))

        self.get_logger().info(
            f"temp={temp:.1f}, humidity={humidity:.1f}, wind={wind_speed:.1f}, dir={wind_direction:.0f}"
        )

        self.slot_idx = (self.slot_idx + 1) % self.num_slots


def main(args=None):
    rclpy.init(args=args)
    node = WeatherSplitPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
