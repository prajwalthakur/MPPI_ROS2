import jax
import jax.numpy as jnp
import matplotlib.pyplot as plt
from matplotlib import patches, lines
from matplotlib.ticker import MultipleLocator, AutoMinorLocator
import pdb
import numpy as np
class vis_dynamics:
    def __init__(self,pose_lim,start,goal,obs_pose,robot_r,obs_r,noise_std_dev,noise_max_limit,rnd_key):
        self.num_state_dim:int = 2
        self.num_control_dim:int  = 2 
        self.init_pose = start #2x1 array
        self.goal_pose = goal  #2x1 array
        self.key = rnd_key
        self.noise_key,_ = jax.random.split(self.key)
        self.noise_std_dev = noise_std_dev
        self.noise_max_limit = noise_max_limit
        self.obs_pose = obs_pose  #num_obsx2x1 array
        self.robot_r = robot_r
        self.obs_r = obs_r
        self.pose_lim = pose_lim
        self.horizon_length  = 10
        self.num_mppi_rollout = 100
        self.plot_itr = 0
        self.init_visualization()
        
    def sample_noise(self):
        self.noise_key,subkey = jax.random.split(self.noise_key)
        noise = jax.random.normal(subkey,shape=((self.num_control_dim,1)))
        scaled_noise = noise * self.noise_std_dev
        clipped_noise = jnp.clip(scaled_noise, -self.noise_max_limit, self.noise_max_limit)
        return clipped_noise
        
    def step(self,optimal_control, X_optimal_seq,X_rollout):
        
        self.X_optimal_seq = X_optimal_seq
        self.X_rollouts = X_rollout
        #check if the dimension of the control is appropriate     
        control = optimal_control 
        control  = jnp.reshape(control,(self.num_control_dim,1))
        assert control.shape == (self.num_control_dim,1)
        #state dynamics x[k+1] = x[k] + u[k] +noise
        noise_sampled = self.sample_noise()
        self.init_pose =  self.init_pose + control + noise_sampled
        self.step_visualization()
        return self.init_pose

    def init_visualization(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(self.pose_lim[0,0], self.pose_lim[1,0])
        self.ax.set_ylim(self.pose_lim[0,1], self.pose_lim[1,1])
        self.ax.xaxis.set_major_locator(MultipleLocator(0.4))
        self.ax.yaxis.set_major_locator(MultipleLocator(0.4))

        
        start_patch = patches.Circle((self.init_pose[0,0], self.init_pose[1,0]),radius=self.robot_r, fc='green', alpha=0.5,label="Start pose" )
        self.ax.add_patch(start_patch )
 
 
        goal_patch = patches.Circle((self.goal_pose[0,0], self.goal_pose[1,0]), radius=self.robot_r, fc='red', alpha=0.5,label="goal pose")
        self.ax.add_patch(goal_patch)
        
               
        self.robot_patch = patches.Circle((self.init_pose[0,0], self.init_pose[1,0]), radius=self.robot_r, fc='blue', alpha=0.7,label="robot pose")
        
        self.ax.add_patch(self.robot_patch)
        
        
        for i in range(self.obs_pose.shape[0]):
            label = "Obstacle" if i == 0 else "_nolegend_"
            obs_patch = patches.Circle((self.obs_pose[i][0] + self.obs_r ,self.obs_pose[i][1]+ self.obs_r), radius=self.obs_r, fc='black', alpha=1.0,label=label)
            self.ax.add_patch(obs_patch)   
        self.ax.set_title('MPPI ')
        self.ax.set_aspect('equal') 
        #self.ax.tick_params(axis='x', which='both', labelbottom=False)      
        self.ax.grid(True,)
        # • horizon (optimal sequence) – thick orange line
        self.opt_line, = self.ax.plot([], [], lw=3, color='red',
                                    label='optimal predicted path')

        # • perturbed roll‑outs – very light grey for all 100
        self.rollout_lines = []
        for i in range(self.num_mppi_rollout):
            label = "MPPI rollouts" if i == 0 else "_nolegend_"
            line, = self.ax.plot([], [], lw=1, color='grey', alpha=0.30,label=label)
            self.rollout_lines.append(line)   
        
        label = "traced robot path" if self.plot_itr == 0 else "_nolegend_"
        self.path_patch = patches.Circle((self.init_pose[0,0], self.init_pose[1,0]), radius=0.03, fc='purple', alpha=1.0,label= label)
        self.ax.add_patch(self.path_patch)
        self.ax.legend(loc="upper right", framealpha=0.9)
        
    def step_visualization(self):
        
        self.robot_patch.center = (self.init_pose[0,0], self.init_pose[1,0])
        
        self.path_patch = patches.Circle((self.init_pose[0,0], self.init_pose[1,0]), radius=0.03, fc='purple', alpha=1.0,label= "_nolegend_")
        
        self.ax.add_patch(self.path_patch)
        # -- update *optimal* horizon ----------------------------------
        #   u_optimal_seq   : (H, 2)  (JAX array)
        #pdb.set_trace()
        path_xy = np.asarray(self.X_optimal_seq)    
        self.opt_line.set_data(path_xy[:, 0], path_xy[:, 1])

        # -- update *perturbed* roll‑outs ------------------------------
        #   perturbed_controls : (R, H, 2)  (JAX array)
        rollouts_xy = np.asarray(self.X_rollouts)
        for i, line in enumerate(self.rollout_lines):
            traj = rollouts_xy[i]                     # (H, 2)
            line.set_data(traj[:, 0], traj[:, 1])
        
        
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        self.plot_itr+=1
    
        
        
    
    
    
    
    
