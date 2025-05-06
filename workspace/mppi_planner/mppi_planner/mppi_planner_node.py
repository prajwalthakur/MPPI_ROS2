#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path, Odometry
from tf_transformations import euler_from_quaternion  


from mppi_planner.mppi_class import MPPI
import jax
import jax.numpy as jnp
import numpy as np
import yaml
import time

#configuration file
CONFIG_PATH = 'src/mppi_planner/config/sim_config.yaml'
# Load core parameters from YAML
with open(CONFIG_PATH, 'r') as f:
    cfg = yaml.safe_load(f)

seed                     = int(cfg['seed'])
goal                     = jnp.array(cfg['goal'])
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
pose_lim                 = jnp.array(cfg['pose_lim'])
obs_array                = jnp.array(cfg['obs_array'])
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

### set goal
GOAL = goal
####

class SimplePlanner(Node):
    def __init__(self):
        super().__init__('mppi_planner_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.goal = GOAL
        self.init = False
        self.current_pose = None
        timer_period = dt  # 1/(planning rate) currently ~ 10hz
        
        # timer to call planning function
        self.control_timer = self.create_timer(timer_period, self.control_cb)
        
        # visualization utility
        self.path_pub = self.create_publisher(Path,  '/mppi/robot_path',   10)
        self.opt_pub  = self.create_publisher(Marker,  '/mppi/opt_rollout',     10)
        self.roll_pub = self.create_publisher(Marker,'/mppi/rollouts',     10)
        self.robot_path = Path()
        self.robot_path.header.frame_id = 'odom'

    def odom_callback(self, msg: Odometry):
        """Callback to handle incoming odometry messages."""
        current_pose = msg.pose.pose
        q = current_pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]

        # Convert to Euler angles
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.get_logger().debug(
            f'Received odom: position=({current_pose.position.x:.2f}, '
            f'{current_pose.position.y:.2f})'
        )
        self.current_pose  = jnp.array([current_pose.position.x,current_pose.position.y,yaw])
        # Create a deterministic PRNGKey from our fixed seed so that MPPI sampling is reproducible.
        # Then split it into two independent keys:
        #  - mppi_key: used for generating control perturbations
        #  - goal_key: reserved for any goal‐related randomness
        key = jax.random.PRNGKey(seed)
        mppi_key,goal_key  = jax.random.split(key, 2)
        if self.init is False:
            start =  self.current_pose
            self.MppiObj  = MPPI(start,mppi_key)
            self.init = True
            
    # function to plot traced path, optimal predicted path and MPPI trajectory rollouts        
    def publish_path_utils(self, X_optimal_seq, X_rollout, num_to_vis=20):
        now = self.get_clock().now().to_msg()

        # 1) Robot’s past path
        self.robot_path.header.stamp = now
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = 'odom'
        ps.pose.position.x = float(self.current_pose[0])
        ps.pose.position.y = float(self.current_pose[1])
        ps.pose.orientation.w = 1.0
        self.robot_path.poses.append(ps)
        self.path_pub.publish(self.robot_path)

        
        opt_marker = Marker()
        opt_marker.header.stamp = now
        opt_marker.header.frame_id = 'odom'
        opt_marker.ns = 'mpii_optimal_rollout'
        opt_marker.id = 0
        opt_marker.type = Marker.LINE_LIST
        opt_marker.action = Marker.ADD
        opt_marker.scale.x = 0.02  
        opt_marker.color.r = 1.0
        opt_marker.color.g = 0.0
        opt_marker.color.b = 0.0
        opt_marker.color.a = 1.0        
        for i in range(X_optimal_seq.shape[0] - 1):
            p0 = X_optimal_seq[i,:]
            p1 = X_optimal_seq[i+1,:]
            opt_marker.points.append(Point(x=float(p0[0]), y=float(p0[1]), z=0.0))
            opt_marker.points.append(Point(x=float(p1[0]), y=float(p1[1]), z=0.0))        
        self.opt_pub.publish(opt_marker)

        #  MPPI rollouts as a Marker (LINE_LIST)
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = 'odom'
        marker.ns = 'mpii_rollouts'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.1

        # Only visualize num_to_vis rollouts
        step = int(X_rollout.shape[0]/num_to_vis)
        for itr in  range(0,X_rollout.shape[0],step):
            rollout = X_rollout[itr]
            # rollout is [H x 2], draw segments between consecutive points
            for i in range(rollout.shape[0] - 1):
                p0 = rollout[i]
                p1 = rollout[i+1]
                marker.points.append(Point(x=float(p0[0]), y=float(p0[1]), z=0.0))
                marker.points.append(Point(x=float(p1[0]), y=float(p1[1]), z=0.0))

        self.roll_pub.publish(marker)    
        
    def control_cb(self):
        twist = Twist()
        print("in control cb")
        X_optimal_seq = np.zeros((horizon_length,dim_ctrl))
        X_rollout = np.zeros((mppi_num_rollouts,horizon_length,dim_ctrl))
        if self.init is True:
            dist_to_goal = jnp.linalg.norm(self.current_pose[0:dim_euclid] - self.goal[0:dim_euclid])
            if( dist_to_goal<= goal_tolerance):
                optimal_control = np.zeros((dim_ctrl,1))
            else:
                start = time.time()
                optimal_control, X_optimal_seq,X_rollout = self.MppiObj.compute_control(self.current_pose,self.goal)
                self.get_logger().info(f'time took to compute control commands ={time.time()-start}')
            self.publish_path_utils(X_optimal_seq,X_rollout,num_to_vis=20)
            twist.linear.x = optimal_control[0][0]
            twist.angular.z = optimal_control[1][0]
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f'Published cmd_vel: linear.x={twist.linear.x:.2f},angular.z={twist.angular.z:.2f}')
            self.get_logger().info(f'euclid dist to goal={dist_to_goal:.2f}')
        

def main():
    rclpy.init()
    node = SimplePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
