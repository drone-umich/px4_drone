import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from new_interfaces.msg import ExampleString

class NodeName(Node):

    def __init__(self) -> None:
        super().__init__('node_name')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers

        # Create subscribers
        self.subscriber = self.create_subscription(ExampleString,"example_topic",self.subscriber_callback,qos_profile)

        # Initialize variables

        # Create a timer

    # Create callbacks for subscribers and timers
    def subscriber_callback(self,msg_received):
        msg = msg_received.data
        self.get_logger().info(msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    node_name = NodeName()
    rclpy.spin(node_name)
    node_name.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
