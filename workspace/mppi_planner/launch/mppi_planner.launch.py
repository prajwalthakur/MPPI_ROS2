import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

##@@ Launch file to spawn turtlebot in an empty world and cylindrical obstacles
def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('mppi_planner'), 'config')
    rviz_config_dir = os.path.join(config_dir, 'mppi_rviz.rviz')
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=os.path.join(config_dir, 'tb3_map.yaml'),
        description='Path to the map YAML file',
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('turtlebot3_gazebo'), '/launch', '/empty_world.launch.py'])
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
    )
    spawn_cylinders_node = Node(
        package='mppi_planner',
        executable='spawn_cylinder',
        output='screen',
    )    
    
    return LaunchDescription([
        declare_map_yaml_cmd,
        gazebo,
        rviz_node,
        spawn_cylinders_node
    ])