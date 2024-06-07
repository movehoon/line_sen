import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes

from std_msgs.msg import UInt8MultiArray

import can

class PcanManager(Node):

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('pcan_manager')
        try:
            self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)
        except:
            self.bus = None
            self.get_logger().error(f'Cannot open PCAN')

        self.pcan_send(0x123, [0, 2, 3, 4, 5, 1, 2, 3])

        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'pcan_send',
            self.pcan_callback,
            10)
        self.subscription

        self.timer = self.create_timer(0.1, self.timer_callback)


    def timer_callback(self):
        try:
            msg = self.bus.recv(0.01)
            if msg is not None:
                data_str = ''.join(':{:02x}'.format(x) for x in msg.data)
                self.get_logger().info(f'Get PCAN id={msg.arbitration_id}, data={data_str}')
        except can.CanError:
            self.get_logger().error(f'CAN recv error')

    def pcan_callback(self, msg):
        self.pcan_send(0x184, msg.data)


    def pcan_send(self, id, _data, len=8):
        can_msg = can.Message(arbitration_id=id, data=_data)
        try:
            self.bus.send(can_msg)
        except can.CanError as e:
            self.get_logger().error(f'PCAN can not send{0}', e)





def main(args=None):
    print('Hi from pcan_manager.')
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    pcan_manager = PcanManager()

    # Spin the node so the callback function is called.
    rclpy.spin(pcan_manager)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pcan_manager.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()

