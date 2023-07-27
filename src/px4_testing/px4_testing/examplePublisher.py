import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from new_interfaces.msg import ExampleString

class ExamplePublisherNode(Node):

    def __init__(self) -> None:
        super().__init__('example_publisher_node')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.publisher = self.create_publisher(ExampleString,"example_topic",qos_profile)

        # Create subscribers

        # Initialize variables

        # Create a timer
        self.timer = self.create_timer(1.0,self.timer_callback)

    # Create callbacks for subscribers and timers
    def timer_callback(self):
        msg = ExampleString()
        msg.data = "test"
        self.publisher.publish(msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExamplePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
