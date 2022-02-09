import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import launch_ros
import os

def generate_launch_description():
    diff_drive_node = Node(
        package = "morbo",
        executable = "morbo_diff_drive"
    )

    rplidar_node = Node(
        node_name='rplidar_composition',
        package='rplidar_ros',
        node_executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 115200,
            'frame_id': 'base_scan',
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    pkg_share = launch_ros.substitutions.FindPackageShare(package='morbo_bringup').find('morbo_bringup')
    default_model_path = os.path.join(pkg_share, 'urdf/morbo.urdf')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
            description='Absolute path to robot urdf file'),
        diff_drive_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rplidar_node,
    ])
