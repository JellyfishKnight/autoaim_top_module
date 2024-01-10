import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('autoaim_bring_up'), 'launch'))
from launch.substitutions import Command


def generate_launch_description():

    from launch_ros.actions import Node
    from launch import LaunchDescription

    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('autoaim_bring_up'), 'config', 'launch_params.yaml')))

    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('autoaim_bring_up'), 'descriptions', 'urdf', 'gimbal_description.urdf.xacro'),
        ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                    'publish_frequency': 1000.0}]
    )

    node_params = os.path.join(
        get_package_share_directory('autoaim_bring_up'), 'config', 'node_params.yaml')

    camera_node = Node(
        package='mindvision_camera',
        executable='mindvision_camera_node',
        name='camera_node',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
    )


    tracker_node = Node(
        package='predictor_node',
        executable='predictor_node_node',
        name='predictor_node',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        ros_arguments=['--log-level', 'predictor_node:='+launch_params['predictor_log_level']],
    )

    detector_node = Node(
        package='detector_node',
        executable='detector_node_node',
        name='detector_node',
        emulate_tty=True,
        output='both',
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                   'armor_detector:='+launch_params['detector_log_level']],
    )

    node_tf2= Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        output='screen',
        arguments = ["0", "0", "0", "0", "0", "0", "odom", "gimbal_link"]
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge'
    )

    autoaim_debugger = Node(
        package='autoaim_debugger',
        executable='autoaim_debugger_node',
        name='autoaim_debugger_node',
        output='both',
        emulate_tty=True,
    )


    return LaunchDescription([
        robot_state_publisher,
        camera_node,
        detector_node,
        tracker_node,
        node_tf2,
        autoaim_debugger,
        foxglove_bridge,
    ])

