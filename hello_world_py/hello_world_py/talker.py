import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self._publisher = self.create_publisher(String, 'chatter', 10)
        self._timer = self.create_timer(0.5, self.timer_callback)
        self._i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self._i
        self._publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self._i += 1

def main(args=None):
    rclpy.init(args=args)

    talker = Talker()

    rclpy.spin(talker)
    
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
