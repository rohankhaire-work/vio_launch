from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
import yaml
from ament_index_python.packages import get_package_share_directory


def load_transforms_from_yaml(context, *args, **kwargs):
    setup_type = LaunchConfiguration('dataset').perform(context)

    # Path to the installed config file
    package_share = get_package_share_directory('vio_launch')
    config_path = os.path.join(
        package_share, 'config', 'static_transforms.yaml')

    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)

    tf_nodes = []
    for tf in data.get(setup_type, []):
        x, y, z = tf['translation']
        roll, pitch, yaw = tf['rotation']
        parent = tf['parent_frame']
        child = tf['child_frame']

        tf_nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{child}_tf',
                arguments=[
                    str(x), str(y), str(z),
                    str(roll), str(pitch), str(yaw),
                    parent, child
                ]
            )
        )

    return tf_nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'dataset',
            default_value='euroc',
            description='Type of dataset: euroc, tum-vi, ntu'
        ),
        OpaqueFunction(function=load_transforms_from_yaml),

        # Play bag file
        DeclareLaunchArgument(
            'bag_path',
            default_value='/home/rohan/kitti_bag_seq_00',
            description='Path to the ROS 2 bag'
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path')],
            output='screen'
        ),

        # Add RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
