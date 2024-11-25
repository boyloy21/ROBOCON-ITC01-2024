#!/usr/bin/env python3
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes

from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

from sensor_msgs.msg import Image 
class ImagePublisher(Node):
  def __init__(self):
    super().__init__('image_publisher')
    self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.cap = cv2.VideoCapture(0)
    self.br = CvBridge()
  def timer_callback(self):
    ret, frame = self.cap.read()        
    if ret == True:
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
    self.get_logger().info('Publishing video frame')
  
def main(args=None):
  rclpy.init(args=args)
  image_publisher = ImagePublisher()
  rclpy.spin(image_publisher)
  image_publisher.destroy_node()
  rclpy.shutdown()
if __name__ == '__main__':
  main()

# if __name__ == '__main__':
#   rclpy.init(args=None)
#   image_publisher = ImagePublisher()
#   rclpy.spin(image_publisher)
#   image_publisher.destroy_node()
#   rclpy.shutdown()