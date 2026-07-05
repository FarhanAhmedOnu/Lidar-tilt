import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Start the RPLidar C1 using its OFFICIAL launch file (Guaranteed to work)
    rplidar_pkg = get_package_share_directory('rplidar_ros')
    c1_launch = os.path.join(rplidar_pkg, 'launch', 'rplidar_c1_launch.py')
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(c1_launch)
    )

    # 2. Start our IMU Node
    imu_node = Node(
        package='sweep_slam_pkg',
        executable='imu_publisher_node',
        name='arduino_imu_node',
        parameters=[{
            'port': '/dev/ttyACM0',      # Verify this is your Arduino port
            'baudrate': 250000,
            'calib_file': os.path.join(os.path.expanduser('~'), 'imu_calibration.json')
        }],
        output='screen'
    )

    return LaunchDescription([
        lidar_node,
        imu_node
    ])