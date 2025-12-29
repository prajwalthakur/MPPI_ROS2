#!/usr/bin/env python3
#@@ script for spwaning cylindrical obstacles in gazebo
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import yaml
import numpy as np

# A simple SDF mppi_docker for a cylinder of given radius and height
# Updated Template for Option B
CYLINDER_SDF = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>false</static>
    <link name='{name}_link'>
      <gravity>false</gravity>
      <inertial>
        <mass>5.0</mass>
        <inertia><ixx>0.083</ixx><iyy>0.083</iyy><izz>0.083</izz></inertia>
      </inertial>

      <collision name='collision'>
        <geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>
      </collision>
      <visual name='visual'>
        <geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>
      </visual>
    </link>

    <plugin name="planar_move" filename="libgazebo_ros_planar_move.so">
      <ros>
        <namespace>/{name}</namespace>
      </ros>
      <bodyName>{name}_link</bodyName>
      <update_rate>50.0</update_rate>
      
      
      <odometryFrame>{name}/odom</odometryFrame>
      <robot_base_frame>{name}_link</robot_base_frame>
      
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
    </plugin>
  </model>
</sdf>
"""
#<odometry_frame>world</odometry_frame>
class CylinderSpawner(Node):
    def __init__(self, positions, radius=0.2, height=1.0):
        super().__init__('cylinder_spawner')
        # Create a client for the /spawn_entity service (gazebo_msgs/srv/SpawnEntity) :contentReference[oaicite:0]{index=0}
        self.cli = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        # Loop over each (x,y) in the provided list
        for idx, (x, y) in enumerate(positions):
            model_name = f'obstacle_{idx}'
            sdf = CYLINDER_SDF.format(name=model_name, radius=radius, height=height)

            # Fill out the request
            req = SpawnEntity.Request()
            req.name              = model_name
            req.xml               = sdf
            req.robot_namespace   = model_name
            req.initial_pose      = Pose()
            req.initial_pose.position.x = x
            req.initial_pose.position.y = y
            req.initial_pose.position.z = height / 2.0
            req.reference_frame   = 'world'

            # Call the service and wait for the result :contentReference[oaicite:1]{index=1}
            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            if res.success:
                self.get_logger().info(f'Spawned {model_name} at ({x}, {y})')
            else:
                self.get_logger().error(
                    f'Failed to spawn {model_name}: {res.status_message}'
                )


with open('src/mppi_planner/config/sim_config.yaml', 'r') as f:
    cfg = yaml.safe_load(f)
obs_array  = np.array(cfg['obs_array'])
obs_radius = cfg['obs_r']
obs_height = cfg['obs_h']




def main(args=None):
    rclpy.init(args=args)

    # N cylinders at these (x,y) positions
    positions = obs_array 

    spawner = CylinderSpawner(positions, radius=obs_radius, height=obs_height)
    # Once all cylinders are spawned, shut down
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
