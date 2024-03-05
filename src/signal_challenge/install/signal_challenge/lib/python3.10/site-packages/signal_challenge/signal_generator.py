import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np
import yaml
import sys

class SignalGeneratorNode(Node):
    def __init__(self):
        super().__init__('signal_generator')

        # Load parameters from params.yaml
        default_params_file = '/home/arturomurra/FundamentacionRob/src/signal_challenge/config/params.yaml'
        self.declare_parameter('params_file', default_params_file)
        params_file = self.get_parameter('params_file').value
        
        with open(params_file, 'r') as file:
            self.params = yaml.safe_load(file)

        # Correctly parsing command-line argument for signal type if provided
        if len(sys.argv) > 1:
            try:
                self.signal_type = int(sys.argv[1])
                self.get_logger().info(f"Command-line signal type set to: {self.signal_type}")
            except ValueError:
                self.get_logger().error("The provided signal type argument is not an integer. Using default from YAML.")
                self.signal_type = self.params.get('type', 0)
        else:
            self.signal_type = self.params.get('type', 0)
            self.get_logger().info(f"No command-line argument. Using default signal type from YAML: {self.signal_type}")

        # Assuming parameters for signals are correctly loaded from YAML
        # You might need to adjust this part if the structure of your YAML file is different
        self.signals = self.params.get('signals', [])

        # Create publisher
        self.publisher = self.create_publisher(Float32, '/signal', 10)

        # Timer to publish signals at 1kHz
        self.timer_period = 1.0 / 1000.0  # Adjust frequency as necessary
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        # Assuming 'self.signals' is a list of dictionaries, each with keys for type, amplitude, frequency, offset
        # If 'self.signal_type' is set, generate and publish signal accordingly
        for signal in self.signals:
            if signal["type"] == self.signal_type:
                amplitude = signal["amplitude"]
                frequency = signal["frequency"]
                offset = signal["offset"]
                value = self.generate_signal_value(amplitude, frequency, offset)
                msg = Float32()
                msg.data = value
                self.publisher.publish(msg)
                break  # Assuming you only want to generate signal for the first match

    def generate_signal_value(self, amplitude, frequency, offset):
        # Example implementation for a sine wave signal
        if self.signal_type == 1:
            value = amplitude * np.sin(2 * np.pi * frequency * self.timer_period) + offset
            self.get_logger().info(f"Signal Type: sine, Value: {value}\n")

        # Add additional signal type cases as needed
        elif self.signal_type == 2:
            value = amplitude * np.sign(np.sin(2 * np.pi * frequency * self.timer_period)) + offset
            self.get_logger().info(f"Signal Type: square, Value: {value}\n")

        elif self.signal_type == 3:
            value = amplitude * (2 * (frequency * self.timer_period - np.floor(0.5 + frequency * self.timer_period))) + offset
            self.get_logger().info(f"Signal Type: sawtooth, Value: {value}\n")

        elif self.signal_type == 4:
            value = amplitude * (2 * np.abs(2 * (frequency * self.timer_period - np.floor(0.5 + frequency * self.timer_period))) - 1) + offset
            self.get_logger().info(f"Signal Type: triangular, Value: {value}\n")

        elif self.signal_type == 5:
            value = 30.0 #amplitude * np.cos(2 * np.pi * frequency * self.timer_period) + offset
            self.get_logger().info(f"Signal Type: cosine, Value: {value}\n")
        else:
            value = 0  # Default or error value
            self.get_logger().info(f"Signal Type: {self.signal_type}, Value: {value}\n")
        return value

def main(args=None):
    rclpy.init(args=args)
    node = SignalGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from math import sin, pi

class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator')
        self.publisher_signal = self.create_publisher(Float32, '/signal', 10)
        self.publisher_time = self.create_publisher(Float32, '/time', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1s period (10 Hz)
        self.time = 0.0

    def timer_callback(self):
        signal_msg = Float32()
        time_msg = Float32()

        signal_value = self.generate_sine_wave()
        signal_msg.data = signal_value

        time_msg.data = self.time

        self.publisher_signal.publish(signal_msg)
        self.publisher_time.publish(time_msg)

        self.get_logger().info(f'Time: {time_msg.data}, Signal: {signal_msg.data}')

        self.time += 0.1  # Increment time by 0.1 seconds (10 Hz)

    def generate_sine_wave(self):
        amplitude = 1.0
        frequency = 1.0
        return amplitude * sin(2 * pi * frequency * self.time)

def main(args=None):
    rclpy.init(args=args)
    signal_generator = SignalGenerator()
    rclpy.spin(signal_generator)
    signal_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''