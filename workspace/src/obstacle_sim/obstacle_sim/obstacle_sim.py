#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path, Odometry
from tf_transformations import euler_from_quaternion  

import numpy as np
import yaml
import time

#configuration file
CONFIG_PATH = 'src/mppi_planner/config/sim_config.yaml'
# Load core parameters from YAML
with open(CONFIG_PATH, 'r') as f:
    cfg = yaml.safe_load(f)

seed                     = int(cfg['seed'])
dt                       = float(cfg['dt'])
robot_r                  = float(cfg['robot_r'])
dim_st                   = int(cfg['dim_st'])
dim_ctrl                 = int(cfg['dim_ctrl'])
obs_r                    = float(cfg['obs_r'])
obs_buffer               = float(cfg['obs_buffer'])
obs_h                    = float(cfg['obs_h'])
goal_tolerance           = float(cfg['goal_tolerance'])
horizon_length           = int(cfg['horizon_length'])
mppi_num_rollouts        = int(cfg['mppi_num_rollouts'])
pose_lim                 = np.array(cfg['pose_lim'])
obs_array                = np.array(cfg['obs_array'])
num_obs                  = int(cfg['num_obs'])
dim_euclid               = int(cfg['dim_euclid'])
noise_std_dev            = float(cfg['noise_std_dev'])
knot_scale               = int(cfg['knot_scale'])
degree                   = int(cfg['degree'])
beta                     = float(cfg['beta'])
beta_u_bound             = float(cfg['beta_u_bound'])
beta_l_bound             = float(cfg['beta_l_bound'])
param_exploration        = float(cfg['param_exploration'])
update_beta              = bool(cfg['update_beta'])
sampling_type            = cfg['sampling_type']
collision_cost_weight    = float(cfg['collision_cost_weight'])
stage_goal_cost_weight   = float(cfg['stage_goal_cost_weight'])
terminal_goal_cost_weight= float(cfg['terminal_goal_cost_weight'])


boundary_eps  = float(cfg['boundary_eps'])
obs_cmd_noise_std_dev = float(cfg['obs_cmd_noise'])
obs_v_min = float(cfg['obs_v_min'])
obs_v_max = float(cfg['obs_v_max'])

class GaussianNoiseSampler:
    def __init__(self, mean, cov, seed=None):
        self.rng = np.random.default_rng(seed)
        self.mean = np.asarray(mean)
        self.cov = np.asarray(cov)

    def sample(self):
        return self.rng.multivariate_normal(self.mean, self.cov)

class simpleObstacleDynamics(Node):
    def __init__(self):
        super().__init__("obstacle_dynamics_node")

        self.num_obs = num_obs
        self.minX, self.minY = pose_lim[0]
        self.maxX, self.maxY = pose_lim[1]

        self.cmd_vel_pub = {}
        self.obs_states = {}
        self.samplers = {}
        self.set_init_vel = False
        self.override = False
        for i in range(self.num_obs):
            name = f'cylinder_{i}'

            self.cmd_vel_pub[name] = self.create_publisher(
                Twist, f'/{name}/cmd_vel', 10
            )

            self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, n=name: self.odom_callback(msg, n),
                10
            )

            self.samplers[name] = GaussianNoiseSampler(
                mean=[0.0, 0.0],
                cov=[[obs_cmd_noise_std_dev**2, 0.0],
                     [0.0, obs_cmd_noise_std_dev**2]],
                seed=100 + i
            )

        self.control_timer = self.create_timer(dt, self.control_loop)

    def odom_callback(self, msg: Odometry, name):
        #self.get_logger().info(f"invoked odom {name}")
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear

        self.obs_states[name] = {
            'x': pos.x,
            'y': pos.y,
            'vx': vel.x,
            'vy': vel.y
        }

    def control_loop(self):
        k=0.05
        if(self.set_init_vel==False):
            for name, state in self.obs_states.items():
                x, y = state['x'], state['y']
                noise = self.samplers[name].sample()
                vx = noise[0]
                vy = noise[1]
                cmd = Twist()
                cmd.linear.x = vx
                cmd.linear.y = vy
                #self.get_logger().info(f"invoked control_loop {name} , vx {vx} & vy {vy}")

                self.cmd_vel_pub[name].publish(cmd)
            self.set_init_vel = True
        else :
            for name, state in self.obs_states.items():
                x, y = state['x'], state['y']
                vx, vy = state['vx'], state['vy']

                noise = self.samplers[name].sample()

                if x < self.minX + boundary_eps or x > self.maxX - boundary_eps:
                    vx = -(vx)
                    self.override = True
                else :
                    vx = vx + noise[0]
                if y < self.minY + boundary_eps or y > self.maxY - boundary_eps:
                    vy = -(vy)
                    self.override = True

                else:
                    vy = vy + noise[1]
                
                np.clip(vx,obs_v_min,obs_v_max)
                np.clip(vy,obs_v_min,obs_v_max)
                if(self.override == False):
                    k =0.05
                    for other_name, other in self.obs_states.items():
                        if other_name == name:
                            continue

                        dx = x - other['x']
                        dy = y - other['y']
                        dist2 = dx*dx + dy*dy

                        if dist2 < (2*obs_r)**2:
                            vx += k * dx / dist2
                            vy += k * dy / dist2

                        np.clip(vx,obs_v_min,obs_v_max)
                        np.clip(vy,obs_v_min,obs_v_max)
                
                cmd = Twist()
                cmd.linear.x = vx 
                cmd.linear.y = vy
                #self.get_logger().info(f"invoked control_loop {name} , vx {vx} & vy {vy}")

                self.cmd_vel_pub[name].publish(cmd)


def main():
    rclpy.init()
    node = simpleObstacleDynamics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
