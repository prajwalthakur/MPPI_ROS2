import os
# Jax utility functions
os.environ["XLA_PYTHON_CLIENT_MEM_FRACTION"] = "0.8"  # 0.9 causes too much lag. 
import jax.numpy as jnp
import numpy as np
import jax
from jax import jit,vmap
from jax import config  # Analytical gradients work much better with double precision.
config.update("jax_debug_nans", True)
config.update("jax_enable_x64", True)
config.update('jax_default_matmul_precision', 'high')
from datetime import datetime
import functools
import matplotlib.pyplot as plt


# https://arxiv.org/pdf/2307.09105 
# Utility function needed for Halton Splines instead for better exploration and smoother trajectories
import ghalton
import scipy.special as scsp
import scipy.interpolate as si
import pdb
import copy
import yaml
with open('src/mppi_planner/config/sim_config.yaml', 'r') as f:
    cfg = yaml.safe_load(f)

# === Core parameters from config.yaml ===
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
stage_goal_cost_weight_orient = float(cfg['stage_goal_cost_weight_orient'])
terminal_goal_cost_weight_orient = float(cfg['terminal_goal_cost_weight_orient'])






class MPPI:
    def __init__(self,start,mppi_key):
        self.curr_st = start
        
        self.obs = obs_array   # num of obstacle
        self.num_obs = self.obs.shape[0]
        self.dim_st = dim_st  # state dimension
        self.dim_ctrl = dim_ctrl # control dimension
        self.robot_r = robot_r
        self.obs_r = obs_r
        self.key = mppi_key
        self.control_pert_key,self.key = jax.random.split(self.key,2)
        self.pose_limit = pose_lim
        self.ctrlTs = dt
        self.dim_euclid =  dim_euclid
        self.ctrl_limit = jnp.array(([[0.0 , 0.8 ],[-0.32,0.32]]))
        self.init_mppi()
        


    
    def init_mppi(self):
        # vmap the function to process the arguments in batch (avoid for loop), stage_cost,terminal_cost, dynamics_Step are written for One arguments
        self.stage_cost_vmap = jit(vmap(self.stage_cost,in_axes=(0,0,0,None,None)))
        self.terminal_cost_vmap = jit(vmap(self.terminal_cost,in_axes=(0,0,0,None,None)))
        self.dynamics_step_vmap = jit(vmap(self.dynamics_step,in_axes=(0,)))

        #std-deviation of the perturbation k*sigma = control_limit 
        #if k=1 sigma = control_limit ; 68% of samples within the control-limit
        #k=2 95% of samples within the control-limit
        self.k = 2
        self.control_cov = jnp.diag(np.array([((self.ctrl_limit[0,1]/self.k)**2),
                                              ((self.ctrl_limit[1,1]/self.k)**2)]
                                             ))    # speeed , angular-speed 
        self.control_mean =  jnp.zeros((2,1))
        self.sampling_type = "gaussian_halton"
        self.horizon_length  = horizon_length
        self.mppi_num_rollouts =mppi_num_rollouts
        self.knot_scale = knot_scale
        self.n_knots = self.horizon_length//self.knot_scale
        self.ndims = self.n_knots*self.dim_ctrl
        self.degree = degree
        self.beta = beta
        self.beta_u_bound = beta_u_bound
        self.beta_l_bound = beta_l_bound
        #self.param_exploration = 0.2
        self.update_beta = update_beta
        seed = int(self.key[0]) 
        self.sequencer = ghalton.GeneralizedHalton(self.ndims, seed)        
        self.U_seqs =  jnp.zeros((self.horizon_length,2))  
        
        self.goal_tolerance  = goal_tolerance         # euclidean goal tolerance in meters
        self.goal_tolerance_orient = 0.1   # orientation goal tolerance in radians
        self.obs_buffer = obs_buffer
        self.collision_cost_weight = collision_cost_weight
        self.stage_goal_cost_weight = stage_goal_cost_weight
        self.stage_goal_cost_weight_orient = stage_goal_cost_weight_orient
        self.terminal_goal_cost_weight = terminal_goal_cost_weight
        self.terminal_goal_cost_weight_orient  =terminal_goal_cost_weight_orient
     
    def compute_weights(self,S:jnp.ndarray) -> jnp.ndarray:
        "compute  weights for each rollout"
        #prepare buffer
        rho = jnp.min(S)
        numerator = jnp.exp( (-1.0/self.beta) * (S-rho) )
        #caluclate the denominator , normalizer
        eta = jnp.sum(numerator)
        #calculate weight
        wt = (1.0 / eta) * numerator
        return wt ,eta  

        
    def compute_control(self,curr_st,goal):
        curr_st = curr_st.reshape((self.dim_st,1))
        goal = goal.reshape((self.dim_st,1))
        assert curr_st.shape == (self.dim_st,1) and goal.shape == (self.dim_st,1)
        self.control_pert_key,current_key = jax.random.split(self.key,2)
        predicted_obs_array = self.obs
        goal_pose = goal
        previous_control = self.U_seqs
        
        #@ compute the delta-u perturbations
        delta_u = self.control_pertubations(control_mean=self.control_mean, control_cov= self.control_cov, sampling_type=self.sampling_type,key=current_key)
        #@ compute the num_mppi_rolloutx1 costs
        nominal_mppi_cost , perturbed_control , delta_u = self.cal_nominal_mppi_cost(curr_st,predicted_obs_array,goal_pose,previous_control, delta_u)
        total_cost = nominal_mppi_cost  
        wght ,eta  = self.compute_weights(total_cost)
        temp =  jnp.sum(wght[:,None]*delta_u,axis=0)  #weighted delta-controls 
        temp =  self._moving_average_filter(xx=temp,window_size=5) # smooth out the delta-controls
        
        self.U_seqs +=temp                                # next-control += prev_control + delta-control
        self.U_seqs = jnp.round(self.U_seqs,decimals=4)  # round off the computed controls
        u_filtered = self.control_clip( self.U_seqs, self.ctrl_limit)  # clip the controls between the limits
        self.U_seqs = self.timeShift(u_filtered) # shift the control to the left, i.e receding horizon 
        
        # update the temperature-parameter online
        # https://arxiv.org/pdf/2307.09105  
        if self.update_beta:
            if eta > self.beta_u_bound:
                self.beta = 0.8*self.beta
        elif eta < self.beta_l_bound:
            self.beta = 1.2*self.beta

         # Computing the mppi path rollouts, predicted optimal path
        optimal_control  = u_filtered[0,0:].reshape((self.dim_ctrl,1))
        X_optimal_seq = np.zeros((self.horizon_length+1,self.dim_st))
        X_optimal_seq[0,0:] = curr_st.squeeze()
        X_rollout = np.zeros((self.mppi_num_rollouts,self.horizon_length+1,self.dim_st))
        X_rollout[:,0,0:] = jnp.tile(curr_st.T , (self.mppi_num_rollouts,1))
        for i in range(1,self.horizon_length+1):
            X_optimal_seq[i,0:] = self.dynamics_step_single_pred(X_optimal_seq[i-1,0:].reshape((self.dim_st,1)),u_filtered[i-1,0:].reshape((self.dim_ctrl,1))).squeeze()
            X_rollout[:,i,0:] = self.dynamics_step(X_rollout[:,i-1,0:],perturbed_control[:,i,0:])
        # return  
        return optimal_control, X_optimal_seq,X_rollout
    

    def cal_nominal_mppi_cost(self, predicted_ego_State, predicted_obs_state, goal_array, previous_control_seq, delta_u):
        # for batch rollout the initial state and previous control sequence is tiled for num_mppi_rollouts times
        state_tensor = jnp.tile(predicted_ego_State , (self.mppi_num_rollouts,1,1))
        U_prev_seqs_tensor = jnp.tile(self.U_seqs,(self.mppi_num_rollouts,1,1))
        
        #  param_exploration decides the percentage of exploration (adding control-noise) and  rest without noise
        # Not used in this simulation
        # ref : https://arxiv.org/abs/2209.12842
        #idx_exp : int = int((1-self.param_exploration)*self.mppi_num_rollouts) 
        U_current_tensor = np.zeros((self.mppi_num_rollouts,self.horizon_length,self.dim_ctrl))   
        U_current_tensor= U_prev_seqs_tensor + delta_u
        U_current_tensor = self.control_clip_vec(U_current_tensor,self.ctrl_limit)   
        U_current_tensor = jnp.round(U_current_tensor, decimals=4)
        seq_cost =  jnp.zeros((self.mppi_num_rollouts,1))
        
        #calulate stage and terminal costs ref, similar to : https://arxiv.org/abs/2210.00153
        is_goal_reached = jnp.zeros((self.mppi_num_rollouts,1))
        is_goal_reached_orient = jnp.zeros((self.mppi_num_rollouts,1))
        for itr in range(0,self.horizon_length):

            state_tensor= self.dynamics_step(state_tensor,U_current_tensor[:,itr,:,jnp.newaxis])

           
            stage_cost,is_goal_reached,is_goal_reached_orient = self.stage_cost_vmap(state_tensor,is_goal_reached,is_goal_reached_orient,goal_array,predicted_obs_state)
            seq_cost += stage_cost

        stage_cost = self.terminal_cost_vmap(state_tensor,is_goal_reached,is_goal_reached_orient,goal_array , predicted_obs_state)    
        seq_cost+=stage_cost 
        delta_u = U_current_tensor-U_prev_seqs_tensor
        return seq_cost,U_current_tensor,delta_u
    
    
    
    @functools.partial(jax.jit, static_argnums=0) 
    def dynamics_step(self,st:jnp.array,ut:jnp.array):
        dt = self.ctrlTs
        xdot = ut[0:,0]*jnp.cos(st[0:,2])
        ydot = ut[0:,0]*jnp.sin(st[0:,2])
        omega_dot = ut[0:,1]
        Xdot = jnp.stack((xdot,ydot,omega_dot),axis=1)
        st = st + Xdot*dt
        return st
    
    # differential drive robot
    @functools.partial(jax.jit, static_argnums=0) 
    def dynamics_step_single_pred(self,st:jnp.array,ut:jnp.array):
        st  = st.T
        ut  = ut.T
        dt = self.ctrlTs
        xdot = ut[0:,0]*jnp.cos(st[0:,2])
        ydot = ut[0:,0]*jnp.sin(st[0:,2])
        omega_dot = ut[0:,1]
        Xdot = jnp.stack((xdot,ydot,omega_dot),axis=1)
        st = st + Xdot*dt
        return st.T    
       
    # defination of state-cost and terminal cost, similar to : https://arxiv.org/abs/2210.00153
    #equation 3,4,5,6 
    @functools.partial(jax.jit, static_argnums=0) 
    def stage_cost(self,s_t,is_goal_reached,is_reached_orient,goal_pose,obstacle_array):       
        
        # eulcid distance to the obstacle
        dist_to_obs = jnp.linalg.norm(jnp.asarray([obstacle_array -  s_t[0:self.dim_euclid,0:].T]),axis=-1).T   # num_obs x 1
        cost_obs = jnp.where(dist_to_obs < self.obs_r + self.robot_r + self.obs_buffer,1.0,0)*self.collision_cost_weight
        cost_obs = jnp.sum(cost_obs,axis=0)
        
        # orientation error to goal and associated cost
        abs_error = jnp.abs(s_t[-1,0] - goal_pose[-1] )
        abs_error = jnp.arctan2(jnp.sin(abs_error),jnp.cos(abs_error))
        dist_to_goal_orient = jnp.linalg.norm( abs_error)
        is_reached_orient  = (1-is_reached_orient)*jnp.where(dist_to_goal_orient<=self.goal_tolerance_orient,1.0,0.0)
        cost_to_goal_orient = (1-is_reached_orient)*dist_to_goal_orient*self.stage_goal_cost_weight_orient*1.0 
        
        # euclideand distance to goal and associated cost
        dist_to_goal =    jnp.linalg.norm( s_t[0:self.dim_euclid,0] - goal_pose[0:self.dim_euclid] )
        is_reached = (1-is_goal_reached)*jnp.where(dist_to_goal<=self.goal_tolerance,1.0,0.0)
        cost_to_goal = (1-is_goal_reached)*dist_to_goal*self.stage_goal_cost_weight*1.0 
        
        final_cost = cost_obs + cost_to_goal + cost_to_goal_orient
        return final_cost,is_reached,is_reached_orient
    
    @functools.partial(jax.jit, static_argnums=0)
    def terminal_cost(self,s_t,is_goal_reached,is_reached_orient,goal_pose,obstacle_array):
        # goal tolerance 
        dist_to_goal =    jnp.linalg.norm( s_t[0:self.dim_euclid] - goal_pose[0:self.dim_euclid] )
        dist_to_goal_orient = jnp.linalg.norm( s_t[-1,0] - goal_pose[-1] )
        
        final_terminal_cost = dist_to_goal*(1-is_goal_reached)*self.terminal_goal_cost_weight + dist_to_goal_orient*(1-is_reached_orient)*self.terminal_goal_cost_weight_orient
        return final_terminal_cost   
    
    
    def control_clip_vec(self, v: jnp.ndarray,ctrl_limit) :
        """clamp input"""
        # limit control inputs
        # temp_array =  
        temp_array = np.asarray(v,copy=True)
        temp_array[:,:,0] = jnp.clip(v[:,:,0],  ctrl_limit[0,0],  ctrl_limit[0,1] ) # limit acceleraiton input
        temp_array[:,:,1] = jnp.clip(v[:,:,1],  ctrl_limit[1,0],  ctrl_limit[1,1] ) # limit steering input
        return jnp.asarray(temp_array)
    
    def control_clip(self, ctrl: jnp.ndarray,ctrl_limit) :
        v = np.array(ctrl)
        """clamp input"""
        # limit control inputs
        v[:,0] = jnp.clip(v[:,0],  ctrl_limit[0,0],  ctrl_limit[0,1]) 
        v[:,1] = jnp.clip(v[:,1],  ctrl_limit[1,0],  ctrl_limit[1,1]) 
        return v


    def bspline(self,c_arr, t_arr=None, n=100, degree=3):
        #sample_device = c_arr.device
        sample_dtype = c_arr.dtype
        cv = c_arr

        if(t_arr is None):
            t_arr = np.linspace(0, cv.shape[0], cv.shape[0])
        else:
            t_arr = t_arr
        spl = si.splrep(t_arr, cv, k=degree, s=0.5)
        xx = np.linspace(0, cv.shape[0], n)
        samples = si.splev(xx, spl, ext=3)
        samples = np.array(samples, dtype=sample_dtype)
        return samples   
    def timeShift(self,u_filtered):
        u_temp = copy.deepcopy(u_filtered)
        u_temp[0:-1,0:] = u_temp[1:,0:]
        return u_temp  
      
    #https://arxiv.org/pdf/2307.09105  
    # # https://proceedings.mlr.press/v164/bhardwaj22a.html
    # idea : sample the noise from ghalton for uniform sampling between [0,1],
    # convert it to the gaussian noise using erfinv then use the B-spline to smooth out the control actions
    # for low dimension, a simple gaussian sampler could work
    def control_pertubations(self, control_mean, control_cov,sampling_type,key):
        if sampling_type == "gaussian_halton":
            sample_shape = self.mppi_num_rollouts
            
            knot_points = np.array(self.sequencer.get(self.mppi_num_rollouts),dtype=float)
            # map uniform Halton points → standard normal samples
            gaussian_halton_samples = np.sqrt(2.0)*scsp.erfinv(2 * knot_points - 1)
            # Sample splines from knot points:
            # iteratre over action dimension:
            # reshape to (rollouts, dim_ctrl, n_knots)
            knot_samples = gaussian_halton_samples.reshape(self.mppi_num_rollouts, self.dim_ctrl, self.n_knots) # n knots is T/knot_scale 
            delta_u = np.zeros((sample_shape, self.horizon_length, self.dim_ctrl))
          
            for i in range(sample_shape):
                for j in range(self.dim_ctrl):
                    delta_u[i,:,j] = self.bspline(knot_samples[i,j,:], n=self.horizon_length, degree=self.degree)
        temp =  np.sqrt((control_cov))
        delta_u = np.matmul(delta_u ,temp )
        return delta_u    
    
    def _moving_average_filter(self,xx:np.ndarray,window_size:int)->np.ndarray:
        """apply moving average filter for smoothing input sequence
        Ref. https://zenn.dev/bluepost/articles/1b7b580ab54e95
        """   
        b = np.ones(window_size)/window_size # 1D array of size 10 value = 1/10
        dim = xx.shape[1]  # 3
        xx_mean = np.zeros(xx.shape) #20x3
        for d in range(dim):
            xx_mean[:,d] = np.convolve(xx[:,d] , b , mode = "same")
            n_cov = int(np.ceil(window_size/2))
            for i in range(1,n_cov):
                xx_mean[i,d] *= window_size/(i+n_cov)
                xx_mean[-i,d] *= window_size/(i+n_cov - (window_size % 2))    
        return xx_mean
