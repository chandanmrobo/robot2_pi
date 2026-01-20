#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
import io
import threading


class CameraNode(Node):
    def __init__(self):
        super().__init__('robot2_camera_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('resolution_width', 640),
                ('resolution_height', 480),
                ('framerate', 30),
                ('jpeg_quality', 85),
                ('flip_horizontal', False),
                ('flip_vertical', False),
                ('raw_topic', '/robot2/camera/image_raw'),
                ('compressed_topic', '/robot2/camera/image_raw/compressed'),
                ('enable_preprocessing', True),
                ('brightness', 1.0),
                ('contrast', 1.0)
            ]
        )
        
        # Get parameters
        self.width = self.get_parameter('resolution_width').value
        self.height = self.get_parameter('resolution_height').value
        self.fps = self.get_parameter('framerate').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.flip_h = self.get_parameter('flip_horizontal').value
        self.flip_v = self.get_parameter('flip_vertical').value
        self.raw_topic = self.get_parameter('raw_topic').value
        self.compressed_topic = self.get_parameter('compressed_topic').value
        self.enable_preprocessing = self.get_parameter('enable_preprocessing').value
        self.brightness = self.get_parameter('brightness').value
        self.contrast = self.get_parameter('contrast').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Create publishers
        # Raw image publisher (for local processing if needed)
        self.raw_publisher = self.create_publisher(
            Image,
            self.raw_topic,
            10
        )
        
        # Compressed image publisher (for UI transmission)
        self.compressed_publisher = self.create_publisher(
            CompressedImage,
            self.compressed_topic,
            10
        )
        
        # Initialize camera
        self.camera = None
        self.camera_lock = threading.Lock()
        self.is_running = False
        
        # Start camera
        self.initialize_camera()
        
        self.get_logger().info(f'Camera node initialized - Publishing to {self.compressed_topic}')
        self.get_logger().info(f'Resolution: {self.width}x{self.height} @ {self.fps}fps')
        
    def initialize_camera(self):
        """
        Initialize Pi Camera with specified settings
        """
        try:
            self.camera = Picamera2()
            
            # Configure camera
            camera_config = self.camera.create_still_configuration(
                main={
                    "size": (self.width, self.height),
                    "format": "RGB888"
                },
                controls={
                    "FrameRate": self.fps
                }
            )
            
            self.camera.configure(camera_config)
            self.camera.start()
            
            # Wait for camera to warm up
            import time
            time.sleep(2)
            
            self.is_running = True
            
            # Start capture thread
            self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
            self.capture_thread.start()
            
            self.get_logger().info('Pi Camera initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize camera: {str(e)}')
            self.is_running = False
            
    def capture_loop(self):
        """
        Main capture loop - runs in separate thread
        """
        while self.is_running and rclpy.ok():
            try:
                # Capture frame from camera
                frame = self.camera.capture_array()
                
                if frame is not None:
                    # Process and publish frame
                    self.process_and_publish(frame)
                    
            except Exception as e:
                self.get_logger().error(f'Error in capture loop: {str(e)}')
                
    def process_and_publish(self, frame):
        """
        Process captured frame and publish to ROS topics
        
        Args:
            frame: numpy array containing RGB image
        """
        try:
            # Convert RGB to BGR (OpenCV format)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Apply flipping if configured
            if self.flip_h:
                frame_bgr = cv2.flip(frame_bgr, 1)  # Horizontal flip
            if self.flip_v:
                frame_bgr = cv2.flip(frame_bgr, 0)  # Vertical flip
                
            # Apply preprocessing if enabled
            if self.enable_preprocessing:
                frame_bgr = self.preprocess_frame(frame_bgr)
            
            # Publish compressed image (primary for UI)
            self.publish_compressed(frame_bgr)
            
            # Publish raw image (for local processing)
            self.publish_raw(frame_bgr)
            
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')
            
    def preprocess_frame(self, frame):
        """
        Apply preprocessing to frame (brightness, contrast adjustments)
        
        Args:
            frame: BGR image
            
        Returns:
            Processed BGR image
        """
        # Apply brightness and contrast
        # Formula: new_image = alpha * image + beta
        # alpha controls contrast (1.0-3.0)
        # beta controls brightness (0-100)
        
        alpha = self.contrast
        beta = int((self.brightness - 1.0) * 50)
        
        adjusted = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
        
        return adjusted
        
    def publish_compressed(self, frame):
        """
        Publish compressed JPEG image
        
        Args:
            frame: BGR image to compress and publish
        """
        try:
            # Encode frame as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            result, encoded_image = cv2.imencode('.jpg', frame, encode_param)
            
            if result:
                # Create CompressedImage message
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_frame'
                msg.format = 'jpeg'
                msg.data = encoded_image.tobytes()
                
                # Publish
                self.compressed_publisher.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing compressed image: {str(e)}')
            
    def publish_raw(self, frame):
        """
        Publish raw image (for local processing if needed)
        
        Args:
            frame: BGR image to publish
        """
        try:
            # Convert to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            
            # Publish
            self.raw_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing raw image: {str(e)}')
            
    def cleanup(self):
        """
        Clean up camera resources
        """
        self.is_running = False
        
        if self.camera is not None:
            try:
                self.camera.stop()
                self.camera.close()
                self.get_logger().info('Camera closed successfully')
            except Exception as e:
                self.get_logger().error(f'Error closing camera: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.cleanup()
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()