import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

class VideoPublisher(Node) :
  def __init__(self) :
    super().__init__('VideoPublisher')
    self.publisher = self.create_publisher(Image, '/image_raw', 10)
    time_period = 0.03
    self.timer = self.create_timer(time_period, self.time_callback)
    self.cap = cv2.VideoCapture('/home/robot/Videos/field1.avi') 

  def time_callback(self) :
    ret, frame = self.cap.read()
    # cv2.imwrite('test.jpg', frame)
    if ret == True :
      f = bridge.cv2_to_imgmsg(frame)
      self.publisher.publish(f)
      # cv2.imshow('[video_pub]frame', frame)
      # cv2.waitKey(2)
      self.get_logger().info('Publishing Image')


def main(args=None) :
  rclpy.init(args=args)
  node = VideoPublisher()
  try :
    rclpy.spin(node)
  except KeyboardInterrupt :
    node.get_logger().info('Publish Stopped')
  finally :
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
  main()