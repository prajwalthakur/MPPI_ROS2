#!/usr/bin/env python3
#@@ script for spwaning cylindrical obstacles in gazebo
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import yaml
import numpy as np

# A simple SDF mppi_docker for a cylinder of given radius and height
CYLINDER_SDF = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.7 0.1 0.1 1</ambient>
          <diffuse>0.7 0.1 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

class CylinderSpawner(Node):
    def __init__(self, positions, radius=0.2, height=1.0):
        super().__init__('cylinder_spawner')
        # Create a client for the /spawn_entity service (gazebo_msgs/srv/SpawnEntity) :contentReference[oaicite:0]{index=0}
        self.cli = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        # Loop over each (x,y) in the provided list
        for idx, (x, y) in enumerate(positions):
            model_name = f'cylinder_{idx}'
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
