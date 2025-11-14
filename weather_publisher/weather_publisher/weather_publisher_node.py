import math
import random

import rclpy
from rclpy.node import Node

from weather_interfaces.msg import Weather


class WeatherPublisher(Node):
    def __init__(self):
        super().__init__('weather_publisher')

        # Publisher: topic "weather", queue size 10
        self.publisher_ = self.create_publisher(Weather, 'weather', 10)

        # Timer: publish every 1 second (1 Hz)
        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # 24 h / 15 min = 96 samples
        self.num_slots = 96
        self.slot_idx = 0  # 0..95, loops

        self.get_logger().info(
            'WeatherPublisher node started. '
            'Publishing 1 Hz, each sample = 15 min, 96 samples = 24 h loop.'
        )

    def timer_callback(self):
        """
        Each call represents a 15-minute time step in a synthetic 24-hour cycle.
        Real time: 1 second per step.
        Virtual time: 15 minutes per step.
        """
        msg = Weather()

        # Normalized time-of-day in [0,1)
        # 0.0 = midnight, 0.5 = noon, 1.0 ≈ next midnight
        t_norm = self.slot_idx / float(self.num_slots)

        # ---- Temperature model (°C) ----
        # Base daily cycle: colder at night, warmer in afternoon.
        # Use a cosine so that:
        #   - t_norm = 0.0 (midnight): cooler
        #   - t_norm = 0.5 (noon): warmer
        # Shift by -0.25 so peak is around 14:00–15:00 (mid-afternoon-ish).
        temp_mean = 20.0  # average temp
        temp_amp = 8.0    # day-night swing
        temp = (
            temp_mean
            + temp_amp * math.cos(2.0 * math.pi * (t_norm - 0.25))
            + random.uniform(-0.5, 0.5)  # small noise
        )

        # ---- Humidity model (%) ----
        # Typically higher at night/morning, lower mid-afternoon.
        # Do roughly inverse of temperature.
        hum_mean = 60.0
        hum_amp = 15.0
        humidity = (
            hum_mean
            - hum_amp * math.cos(2.0 * math.pi * (t_norm - 0.25))
            + random.uniform(-2.0, 2.0)
        )
        humidity = max(0.0, min(100.0, humidity))

        # ---- Wind speed model (m/s) ----
        # Slightly breezier during the afternoon.
        wind_base = 1.0
        wind_amp = 3.0
        wind_speed = (
            wind_base
            + wind_amp * max(0.0, math.sin(2.0 * math.pi * (t_norm - 0.25)))
            + random.uniform(-0.5, 0.5)
        )
        wind_speed = max(0.0, wind_speed)

        # ---- Wind direction (deg) ----
        # Let it drift slowly and add some randomness.
        # t_norm * 360 gives a slow rotation over the day.
        wind_direction = (360.0 * t_norm + random.uniform(-20.0, 20.0)) % 360.0

        # Fill the message
        msg.temperature_c = float(temp)
        msg.humidity_percent = float(humidity)
        msg.wind_speed_mps = float(wind_speed)
        msg.wind_direction_deg = float(wind_direction)

        # Publish
        self.publisher_.publish(msg)

        # Log the "virtual time" in hours to make it intuitive
        virtual_minutes = self.slot_idx * 15
        virtual_hours = virtual_minutes / 60.0

        self.get_logger().info(
            f"[t={virtual_hours:4.1f} h] "
            f"T={msg.temperature_c:5.1f} °C, "
            f"H={msg.humidity_percent:5.1f} %, "
            f"Wind={msg.wind_speed_mps:4.1f} m/s @ {msg.wind_direction_deg:6.1f}° "
            f"(slot {self.slot_idx+1}/{self.num_slots})"
        )

        # Advance the slot and wrap at 24h
        self.slot_idx = (self.slot_idx + 1) % self.num_slots


def main(args=None):
    rclpy.init(args=args)
    node = WeatherPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
