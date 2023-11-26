import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('helios_rs_bring_up'), 'launch'))
from launch.substitutions import Command

def generate_launch_description():
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('helios_rs_bring_up'), 'config', 'autoaim', 'launch_params.yaml')))

    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('helios_rs_bring_up'), 'descriptions', 'urdf', 'gimbal_description.urdf.xacro'),
        ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                    'publish_frequency': 1000.0}]
    )

    node_params = os.path.join(
        get_package_share_directory('helios_rs_bring_up'), 'config', 'autoaim', 'node_params.yaml')

    tracker_node = Node(
        package='predictor_node',
        executable='predictor_node_node',
        name='predictor_node',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        ros_arguments=['--log-level', 'predictor_node:='+launch_params['predictor_log_level']],
    )

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
                ComposableNode(
                    package='detector_node',
                    plugin='helios_cv::DetectorNode',
                    name='armor_detector',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                           'armor_detector:='+launch_params['detector_log_level']],
            on_exit=Shutdown(),
        )

    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
    mv_camera_node = get_camera_node('mindvision_camera', 'RMCamera::MVCamera')

    if (launch_params['camera'] == 'hik'):
        cam_detector = get_camera_detector_container(hik_camera_node)
    elif (launch_params['camera'] == 'mv'):
        cam_detector = get_camera_detector_container(mv_camera_node)

    autoaim_bridge_node = Node(
        package='autoaim_bridge',
        executable='autoaim_bridge_node',
        name='autoaim_bridge',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        on_exit=Shutdown(),
        ros_arguments=['--ros-args', '--log-level',
                       'autoaim_bridge:='+launch_params['autoaim_bridge_log_level']],
    )

    delay_autoaim_bridge_node = TimerAction(
        period=1.5,
        actions=[autoaim_bridge_node],
    )

    delay_tracker_node = TimerAction(
        period=2.0,
        actions=[tracker_node],
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge'
    )
    node_tf2= Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        output='screen',
        arguments = ["0", "0", "0", "0", "0", "0", "odom", "gimbal_link"]
    )

    return LaunchDescription([
        robot_state_publisher,
        cam_detector,
        # delay_autoaim_bridge_node,
        delay_tracker_node,
        node_tf2,
        # foxglove_bridge,
        node_tf2,
    ])

