#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

# 이미지 메시지 데이터를 어레이 형태로 변환
bridge = CvBridge() 

class ImageTransfer(Node) :
   def __init__(self) :
    super().__init__('image_sub')
    qos = QoSProfile(depth=10)
    self.publisher = self.create_publisher(Image, '/image_raw', 10)
    time_period = 0.03
    self.image_sub = self.create_subscription(
        Image,
        '/front_stereo_camera/left/image_raw',
        self.image_callback,
        qos)
    self.image = np.empty(shape=[1])
    self.get_logger().info('image_transfer initialized')

   def image_callback(self, data) :
    src = bridge.imgmsg_to_cv2(data, 'bgr8')
    resize_frame = cv2.resize(src, dsize=(640, 480), interpolation = cv2.INTER_AREA)
    f = bridge.cv2_to_imgmsg(resize_frame)
    self.publisher.publish(f)



    # cv2.imshow('img', self.image)
    # cv2.waitKey(33)


     
def main(args=None) :
  rclpy.init(args=args)
  node = ImageTransfer()

  try :
    rclpy.spin(node)
  except KeyboardInterrupt :
    node.get_logger().info('Stopped by Keyboard')
  finally :
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__' :
  main()