from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('mppi_planner'), 'config')
    rviz_config_dir = os.path.join(config_dir, 'navigation.rviz')
    costmap_params_file = LaunchConfiguration('costmap_params_file')
    map_yaml_file = LaunchConfiguration('map_yaml_file')

    declare_costmap_params_cmd = DeclareLaunchArgument(
        'costmap_params_file',
        default_value=os.path.join(config_dir, 'costmap_params.yaml'),
        description='Path to the costmap parameters file',
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=os.path.join(config_dir, 'tb3_map.yaml'),
        description='Path to the map YAML file',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('turtlebot3_gazebo'), '/launch', '/turtlebot3_world.launch.py'])
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml_file}],
        output='screen',
    )
    # Extract origin parameters from the map.yaml (you might need to parse it)
    # For simplicity, let's hardcode the values from your map.yaml here.
    #-1.25, -2.44, 0
    map_origin_x = 0.0
    map_origin_y = 0.0
    map_origin_z = 0.0
    map_origin_roll = 0.0
    map_origin_pitch = 0.0
    map_origin_yaw = 0.0  # Assuming no initial rotation in the map frame

    static_map_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=[str(map_origin_x), str(map_origin_y), str(map_origin_z),
                   str(map_origin_roll), str(map_origin_pitch), str(map_origin_yaw),
                   'world', 'map'],
    )
    map_to_odom_tf_broadcaster_node = Node(
        package='mppi_planner',  # Replace with your package name
        executable='map_to_odom_tf_broadcaster_node',
        name='map_to_odom_tf_broadcaster_node',
        output='screen',
    )
    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        parameters=[costmap_params_file],
        remappings=[('/scan', 'scan')], # Adjust if your scan topic is different
        output='screen',
    )

    global_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        parameters=[costmap_params_file],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        namespace='',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['map_server', 'local_costmap', 'global_costmap']},
        ],
    )
    return LaunchDescription([
        declare_costmap_params_cmd,
        declare_map_yaml_cmd,
        gazebo,
        map_server_node,
        local_costmap_node,
        global_costmap_node,
        lifecycle_manager,
        rviz_node,
    ])