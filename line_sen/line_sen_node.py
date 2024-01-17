import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from geometry_msgs.msg import Twist
import cv2 # OpenCV library
import numpy as np

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      '/front_stereo_camera/left_rgb/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
      
    self.pub_cmdVel = self.create_publisher(Twist, 'cmd_vel', 10)

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    resize_frame = cv2.resize(current_frame, dsize=(360, 240), interpolation = cv2.INTER_AREA)
    val = detect_line(resize_frame)

    h, w, c = resize_frame.shape

    twist = Twist()
    twist.linear.x = 4.0
    twist.angular.z = -(float(val)-w/2)/(w/2)*10
    self.pub_cmdVel.publish(twist)

    # Display image
    cv2.imshow("camera", resize_frame)
    
    cv2.waitKey(1)


def detect_line(image):
    line = 0

    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),0)
    # print(blur[100])

    h, w = blur.shape
    val = np.argmax(blur[int(h/2)])
    print('max: ', val)

    return val




def main(args=None):
    print('Hi from line_sen.')
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
