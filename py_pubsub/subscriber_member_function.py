import rclpy
from rclpy.node import Node

import math

#from std_msgs.msg import Joy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3

#import can
#can.rc['interface'] = 'socketcan'
#can.rc['channel'] = 'can0'
#can.rc['bitrate'] = 500000
#from can import Bus
#bus = Bus()
#bus.receive_own_messages = True

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Twist, 'dolly_cmd', 10)

    def listener_callback(self, msg):
        x = msg.axes[1]
        y = msg.axes[0]
        w = msg.axes[3]
        v = -(((msg.axes[5]+1)/2)-1)
        #msg = can.Message(arbitration_id=0xC0FFEE, data=[0, 25, 0, 1, 3, 1, 4, 1], is_extended_id=True)
        #bus.send(msg)
        msg_out = Twist()
        msg_out.linear = Vector3(x=x , y=y, z=0.0)
        msg_out.angular = Vector3(x=0.0, y=0.0, z=w)
        self.publisher_.publish(msg_out)
        self.get_logger().info('Publishing: "%s"' % msg_out)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
