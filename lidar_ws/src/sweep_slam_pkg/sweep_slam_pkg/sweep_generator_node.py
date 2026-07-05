import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import math

class SweepGeneratorNode(Node):
    def __init__(self):
        super().__init__('sweep_generator_node')
        
        # Declare parameters for the sweep
        self.declare_parameter('min_pwm', 1100) # ~ -45 degrees
        self.declare_parameter('max_pwm', 1900) # ~ +45 degrees
        self.declare_parameter('frequency_hz', 0.1) # 0.1 Hz = one full sweep every 10 seconds
        self.declare_parameter('update_rate_hz', 20.0) # Send commands at 20Hz
        
        self.min_pwm = self.get_parameter('min_pwm').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.freq = self.get_parameter('frequency_hz').value
        self.rate = self.get_parameter('update_rate_hz').value
        
        self.publisher = self.create_publisher(Int32, '/servo_pwm_cmd', 10)
        
        # Timer to publish at 'update_rate_hz'
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f"Sweep Generator Started: {self.min_pwm}us to {self.max_pwm}us at {self.freq}Hz")

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time
        
        # Generate a triangle wave using math.asin(math.sin(...))
        # This creates a smooth, continuous back-and-forth sweep
        phase = 2 * math.pi * self.freq * elapsed
        # Triangle wave oscillates between -1 and 1
        triangle_wave = (2 / math.pi) * math.asin(math.sin(phase)) 
        
        # Map from [-1, 1] to [min_pwm, max_pwm]
        pwm_value = self.min_pwm + (triangle_wave + 1) * 0.5 * (self.max_pwm - self.min_pwm)
        
        msg = Int32()
        msg.data = int(pwm_value)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SweepGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()