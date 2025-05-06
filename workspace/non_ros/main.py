
import os
os.environ["XLA_PYTHON_CLIENT_MEM_FRACTION"] = "0.8"  # 0.9 causes too much lag. 
from datetime import datetime
import functools
# Math
import jax.numpy as jnp
import numpy as np
import jax
from jax import config  # Analytical gradients work much better with double precision.
config.update("jax_debug_nans", True)
config.update("jax_enable_x64", True)
config.update('jax_default_matmul_precision', 'high')

from dyn_visualization import vis_dynamics
from mppi import MPPI
import pdb
import matplotlib.pyplot as plt
import time
import sys
import signal, sys, matplotlib.pyplot as plt
import copy

######### loading configurations
import yaml
# Locate the YAML config file (assumes this script lives in src/mppi_planner/)
CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'config', 'sim_config.yaml')

# Load core parameters from YAML
with open(CONFIG_PATH, 'r') as f:
    cfg = yaml.safe_load(f)

# === Core parameters from sim_config.yaml ===
seed                     = int(cfg['seed'])
num_obs                  = int(cfg['num_obs'])
robot_r                  = float(cfg['robot_r'])
dim_st                   = int(cfg['dim_st'])
dim_ctrl                 = int(cfg['dim_ctrl'])
pose_lim                 = jnp.array(cfg['pose_lim'])
noise_std_dev            = float(cfg['noise_std_dev'])
horizon_length           = int(cfg['horizon_length'])
mppi_num_rollouts        = int(cfg['mppi_num_rollouts'])
knot_scale               = int(cfg['knot_scale'])
degree                   = int(cfg['degree'])
beta                     = float(cfg['beta'])
beta_u_bound             = float(cfg['beta_u_bound'])
beta_l_bound             = float(cfg['beta_l_bound'])
param_exploration        = float(cfg['param_exploration'])
update_beta              = bool(cfg['update_beta'])
sampling_type            = cfg['sampling_type']
goal_tolerance           = float(cfg['goal_tolerance'])
obs_buffer               = float(cfg['obs_buffer'])
collision_cost_weight    = float(cfg['collision_cost_weight'])
stage_goal_cost_weight   = float(cfg['stage_goal_cost_weight'])
state_limit_cost_weight  = float(cfg['state_limit_cost_weight'])
terminal_goal_cost_weight= float(cfg['terminal_goal_cost_weight'])


# === Derived parameters ===
cell_size       = 2 * robot_r
obs_r           = (cell_size / 2.0) + 0.01
noise_max_limit = 2 * noise_std_dev
ctrl_max_limit  = (2**0.5) * robot_r - noise_max_limit
k               = 2
control_cov     = ((ctrl_max_limit / k) ** 2) * jnp.diag(jnp.array([1, 1]))
control_mean    = jnp.zeros((dim_ctrl, 1))
n_knots         = horizon_length // knot_scale
ndims           = n_knots * dim_ctrl
ctrl_limit =  ((2**0.5)*robot_r)  - ( noise_max_limit )  # maximum control limit is 2^{0.5}*robot_r without the noise , with noise its robot_r - noise_max_limit

## for signal.SIGINT
# Interrupt from keyboard (CTRL + C).
def handle_sigint(sig, frame):
    print("CtrlC detected — shutting down.")
    plt.close('all')
    sys.exit(0)
signal.signal(signal.SIGINT, handle_sigint)




## get_start_goal : set the start and goal or set it to sample from random number generator
## set start and goal pose
## can generate this randomly, change seed to change the configuration
def get_start_goal(pose_lim, key)->tuple:
    
    start_key,goal_key  = jax.random.split(key, 2)
    # currently start and goal set to fixed pose
    start_sampled = jax.random.normal(start_key, shape=(2,1))
    goal_sampled = jax.random.normal(goal_key, shape=(2,1))
    start = jnp.array([2.8,-2.8]).reshape((2,1)) #scale_rnd_num(pose_lim,start_sampled)
    goal =  jnp.array([-0.8,0.8]).reshape((2,1))  #scale_rnd_num(pose_lim,goal_sampled)
    return (start,goal)

def scale_rnd_num(lim,rnd_num):
    # squashing the sample in [-1,1]
    rnd_num = jnp.tanh(rnd_num/0.5)
    # transforming the samples to [0,1]
    rnd_num = (rnd_num+1.0)/2.0
    # transfroming the samples to the actual bound 
    un_num =  lim[0,0:][...,jnp.newaxis] + (lim[1,0:][...,jnp.newaxis] - lim[0,0:][...,jnp.newaxis] )*(rnd_num)
    return un_num

#@@ utility function to spawn num_obs random obstacles in grid
def get_obs_pose( pose_lim,key,num_obs):
    #obs_array = jax.random.normal(key, shape=(num_obs,2,1))
    #obs_array = scale_rnd_num(pose_lim,obs_array)
    xmin, ymin = pose_lim[0]
    xmax, ymax = pose_lim[1]

    # Discrete grid coordinates
    x_vals = jnp.arange(xmin, xmax,step= cell_size)
    y_vals = jnp.arange(ymin, ymax,step= cell_size)
    xx, yy = jnp.meshgrid(x_vals, y_vals, indexing='ij')  # shape: (10, 10)
    grid_points = jnp.stack([xx, yy], axis=-1).reshape(-1, 2)  # shape: (100, 2)
    # Sample num_obs unique indices from the flattened grid
    sample_indices = jax.random.choice(key, grid_points.shape[0], shape=(num_obs,), replace=False)
    # Gather the sampled points
    obs_array = grid_points[sample_indices]  # shape: (10, 2)
    return obs_array



if __name__ == "__main__":
    # Create a deterministic PRNGKey from our fixed seed so that MPPI sampling is reproducible.
    # Then split it into two independent keys:
    #  - mppi_key: used for generating control perturbations
    #  - goal_key: reserved for any goal‐related randomness
    key = jax.random.PRNGKey(seed)
    pose_rng,global_key = jax.random.split(key, 2)
    obs_key,global_key = jax.random.split(global_key,2)
    start,goal = get_start_goal(pose_lim=pose_lim, key=pose_rng)
    obs_array:jnp.array = get_obs_pose(pose_lim = pose_lim,key=obs_key,num_obs = num_obs)
    plot_key,global_key = jax.random.split(global_key,2)
    VisualizerObj = vis_dynamics(pose_lim,start,goal,obs_array,robot_r,obs_r,noise_std_dev,noise_max_limit,plot_key)
    plt.ion()
    VisualizerObj.fig.show()
    mppi_key,global_key = jax.random.split(global_key,2)
    MppiObj  = MPPI(pose_lim,start,goal,obs_array,robot_r,obs_r,noise_std_dev,noise_max_limit,dim_st,dim_ctrl,ctrl_limit,mppi_key)
    st = copy.deepcopy(start)
    while(np.linalg.norm(st-goal)>0.1):
        print("dist to gaol",np.linalg.norm(st-goal))
        start = time.time()
        optimal_control, X_optimal_seq,X_rollout= MppiObj.compute_control(st)
        end = time.time()
        print("controller comp. time ", end-start)
        key,curr_key = jax.random.split(key, 2)
        st = VisualizerObj.step(optimal_control, X_optimal_seq,X_rollout )
        print("control computed",optimal_control.T)
    print("goal reached")    
    