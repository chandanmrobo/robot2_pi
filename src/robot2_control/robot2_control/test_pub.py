#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from robot3_msgs.msg import BatteryStatus
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2


class Robot2TestPublisher(Node):
    def __init__(self):
        super().__init__("robot2_test_publisher")
        
        # Publishers - Robot 2 topics
        self.pub_battery = self.create_publisher(
            BatteryStatus, "/robot2/battery_voltage", 10
        )
        self.pub_camera = self.create_publisher(
            CompressedImage, "/robot2/camera/image_raw/compressed", 10
        )
        self.pub_plant = self.create_publisher(
            String, "/robot2/plant_disease", 10
        )
        self.pub_bird = self.create_publisher(
            String, "/robot2/bird_status", 10
        )
        self.pub_move = self.create_publisher(
            String, "/robot2/move_cmd", 10
        )
        
        # Subscribers - From Robot 3
        self.sub_move_cmd = self.create_subscription(
            String, "/robot2/move_cmd", self.cb_move, 10
        )
        self.sub_detection_mode = self.create_subscription(
            String, "/robot2/detection_mode", self.cb_detection_mode, 10
        )
        
        # Test data timers
        self.create_timer(2.0, self.publish_battery)
        self.create_timer(0.1, self.publish_camera)  # 10 FPS
        self.create_timer(5.0, self.publish_detection)
        
        self.battery_value = 90.0
        self.detection_mode = "plant"
        
        self.get_logger().info("Robot 2 Test Publisher Started")
        self.get_logger().info("Publishing test data to Robot 3...")
    
    # ===== PUBLISHERS (Robot 2 → Robot 3) =====
    
    def publish_battery(self):
        """Simulate battery drain"""
        msg = BatteryStatus()
        msg.voltage = self.battery_value / 25.0  # ~3.6V at 90%
        msg.percent = float(self.battery_value)
        msg.low_flag = self.battery_value <= 30.0
        
        self.pub_battery.publish(msg)
        self.get_logger().info(f"📡 Battery: {self.battery_value}%")
        
        # Decrease battery
        self.battery_value -= 0.5
        if self.battery_value < 20.0:
            self.battery_value = 100.0
    
    def publish_camera(self):
        """Simulate camera frames"""
        # Create test pattern
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(frame, "Robot 2 Camera Test", (150, 240),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Encode JPEG
        _, jpeg = cv2.imencode('.jpg', frame)
        
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = jpeg.tobytes()
        
        self.pub_camera.publish(msg)
    
    def publish_detection(self):
        """Simulate detection results"""
        if self.detection_mode == "plant":
            diseases = ["Healthy", "Leaf Blight", "Rust", "Smut"]
            result = diseases[int(self.battery_value) % len(diseases)]
            
            msg = String()
            msg.data = result
            self.pub_plant.publish(msg)
            self.get_logger().info(f"🌱 Plant Detection: {result}")
        
        else:  # bird mode
            birds = ["No Bird", "Crow Detected", "Sparrow Detected"]
            result = birds[int(self.battery_value) % len(birds)]
            
            msg = String()
            msg.data = result
            self.pub_bird.publish(msg)
            self.get_logger().info(f"🐦 Bird Detection: {result}")
    
    # ===== SUBSCRIBERS (Robot 3 → Robot 2) =====
    
    def cb_move(self, msg: String):
        """Receive movement commands"""
        self.get_logger().info(f"⬅️ Movement Command: {msg.data}")
    
    def cb_detection_mode(self, msg: String):
        """Receive detection mode switch"""
        self.detection_mode = msg.data
        self.get_logger().info(f"⬅️ Detection Mode Changed: {self.detection_mode}")


def main():
    rclpy.init()
    node = Robot2TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()