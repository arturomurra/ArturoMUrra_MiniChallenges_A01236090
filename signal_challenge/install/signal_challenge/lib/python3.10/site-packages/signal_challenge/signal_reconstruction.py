# my_signal_generator/my_signal_generator/reconstruction_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ReconstructionNode(Node):
    def __init__(self):
        super().__init__('signal_reconstruction')

        # Create subscriber to the "/signal" topic
        self.subscription = self.create_subscription(
            Float32,
            '/signal',
            self.signal_callback,
            10  # QoS profile depth
        )
        self.subscription  # prevent unused variable warning

    def signal_callback(self, msg):
        # Process the received signal
        reconstructed_value = msg.data  # For simplicity, just pass the received value through

        # Perform your reconstruction logic here if needed

        # Print or log the reconstructed value
        self.get_logger().info(f"Reconstructed Value: {reconstructed_value}")

def main(args=None):
    rclpy.init(args=args)
    node = ReconstructionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
