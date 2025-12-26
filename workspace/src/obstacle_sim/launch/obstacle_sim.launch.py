import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

##@@ Launch file to spawn turtlebot in an empty world and cylindrical obstacles
def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('mppi_planner'), 'config')
    rviz_config_dir = os.path.join(config_dir, 'mppi_rviz.rviz')
    # Path to TurtleBot3 empty world
    tb3_world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'empty_world.world'
    )

    # Start Gazebo WITH factory plugin
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('gazebo_ros'),
    #             'launch',
    #             'gazebo.launch.py'
    #         )
    #     ),
    #     launch_arguments={
    #         'world': tb3_world,
    #         'gui': 'true',
    #         'verbose': 'true'
    #     }.items()
    # )

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            tb3_world
        ],
        output='screen'
    )
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([get_package_share_directory('turtlebot3_gazebo'), '/launch', '/empty_world.launch.py'])
    # )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
    )
    spawn_cylinders_node = Node(
        package='obstacle_sim',
        executable='spawn_cylinder',
        output='screen',
    )    
    return LaunchDescription([
        gazebo,
        rviz_node
    ])