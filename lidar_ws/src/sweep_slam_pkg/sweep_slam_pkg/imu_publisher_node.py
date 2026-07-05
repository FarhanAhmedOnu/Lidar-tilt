import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sweep_slam_pkg.arduino_interface import ArduinoInterface
import os

class ImuPublisherNode(Node):
    def __init__(self):
        super().__init__('arduino_imu_node')
        
        # Declare parameters so they can be changed in the launch file
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 250000)
        # Use an absolute path or ensure the JSON is in the same directory as the node
        self.declare_parameter('calib_file', os.path.join(os.getcwd(), 'imu_calibration.json')) 
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        calib = self.get_parameter('calib_file').value
        
        try:
            self.interface = ArduinoInterface(port, baud, calib)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Arduino Interface: {e}")
            raise
            
        self.publisher = self.create_publisher(Imu, '/imu/data', 100)
        
        # Run at 200Hz (0.005 seconds)
        self.timer = self.create_timer(0.005, self.timer_callback)
        self.get_logger().info("IMU Publisher Node Started Successfully.")

    def timer_callback(self):
        data = self.interface.get_latest_data()
        if data:
            t, accel, gyro = data
            
            msg = Imu()
            # Use ROS 2 clock for the header stamp
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_frame'
            
            # Assign calibrated data (Adafruit already outputs m/s^2 and rad/s)
            msg.linear_acceleration.x = float(accel[0])
            msg.linear_acceleration.y = float(accel[1])
            msg.linear_acceleration.z = float(accel[2])
            
            msg.angular_velocity.x = float(gyro[0])
            msg.angular_velocity.y = float(gyro[1])
            msg.angular_velocity.z = float(gyro[2])
            
            # Set covariances (tune these later based on your Allan Variance)
            msg.linear_acceleration_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]
            msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            
            self.publisher.publish(msg)

    def destroy_node(self):
        """CRITICAL SAFETY HOOK: Ensures serial port is closed on shutdown."""
        self.get_logger().info("Shutting down IMU Node and closing hardware connections...")
        if hasattr(self, 'interface'):
            self.interface.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Node encountered an error: {e}")
    finally:
        # 1. Safely close the hardware serial port
        node.destroy_node()
        
        # 2. Safely shutdown ROS 2 (only if it hasn't been shut down already by the launch system)
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()