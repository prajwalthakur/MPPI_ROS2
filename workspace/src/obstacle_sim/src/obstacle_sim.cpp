#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <yaml-cpp/yaml.h>

#include <unordered_map>
#include <random>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include "../rvo_lib/nav_rvo.h"
#include <memory>
using std::placeholders::_1;



//Gaussian Sampler 
class GaussianNoiseSampler
{
public:
  GaussianNoiseSampler(float stddev, int seed)
  : gen_(seed), dist_(0.0, stddev) {}

  std::pair<float,float> sample()
  {
    return {dist_(gen_), dist_(gen_)};
  }

private:
  std::mt19937 gen_;
  std::normal_distribution<float> dist_;
};



class SimpleObstacleDynamics : public rclcpp::Node
{
public:
  SimpleObstacleDynamics()
  : Node("obstacle_dynamics_node")
  {
    load_config();

    minX_ = pose_lim_[0][0];
    minY_ = pose_lim_[0][1];
    maxX_ = pose_lim_[1][0];
    maxY_ = pose_lim_[1][1];
    std::vector<double> mLimitGoal = {minX_,maxX_,minY_,maxY_};
    mRVO = std::make_shared<RVO::RVOPlanner>("gazebo");
    mRVO->goal_threshold = goalThreshold;
    mRVO->setupScenario(neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, obs_r_, obs_v_max_, mLimitGoal );   // for exp
    
    
    for (int i = 0; i < num_obs_; ++i)
    {
      std::string name = "cylinder_" + std::to_string(i);

      cmd_vel_pubs_[name] =
        create_publisher<geometry_msgs::msg::Twist>(
          "/" + name + "/cmd_vel", 10);

      odom_subs_[name] =
        create_subscription<nav_msgs::msg::Odometry>(
          "/" + name + "/odom",
          10,
          [this, name](nav_msgs::msg::Odometry::SharedPtr msg)
          {
            odom_callback(msg, name);
          });

      samplers_[name] =
        std::make_shared<GaussianNoiseSampler>(
          obs_cmd_noise_std_dev_, 100 + i);
    }
    
    //rvo_goals_init();

    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&SimpleObstacleDynamics::control_loop, this));

    RCLCPP_INFO(get_logger(), "Obstacle dynamics node started");
  }

private:

  void rvo_goals_init()
  {
    mRVO->randGoal("default");
    mRVO->setInitial();
  }


  /* -------- Odom Callback -------- */

  void odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg,
    const std::string & name)
  {
    auto & s = obs_states_[name];
    s.x  = msg->pose.pose.position.x;
    s.y  = msg->pose.pose.position.y;
    s.vx = msg->twist.twist.linear.x;
    s.vy = msg->twist.twist.linear.y;
    if(mRVO->ifAgentExistInmap())
      mRVO->UpdateStateSim(msg, name);
    else
    {
      // Add the agent in the pool
      mRVO->addAgentinSim(msg,name);
      // also set the rand goal for it
      mRVO->setGoalByAgent(msg,name);
    }
  }

  /* -------- Control Loop -------- */

  void control_loop()
  {

    // check if the the agents reached to the curerent goal 
    // if yes then assign new goal
    // compute the speed
    // publish the speed
    // also check if its near the boundary then set a new goal
    for (auto & [name, state] : obs_states_)
    {

      auto noise = samplers_[name]->sample();
      mRVO->setPreferredVelocitiesbyName(name,RVO::Vector2(noise.first, noise.second));
      if(mRVO->isAgentArrived(name))
        mRVO->setGoalByAgent(name,mLimitGoal,"default");
    }

    std::unordered_map<std::string, std::shared_ptr<RVO::Vector2>> new_velocities = mRVO->stepCenteralised();
    for(auto & [name, speed] : new_velocities)
    {
      
      auto vx = speed->x();
      auto vy = speed->y();
      geometry_msgs::msg::Twist cmd;
      // vx = std::clamp(vx, obs_v_min_, obs_v_max_);
      // vy = std::clamp(vy, obs_v_min_, obs_v_max_);
      cmd.linear.x = vx;
      cmd.linear.y = vy;
      cmd_vel_pubs_[name]->publish(cmd);
    }
  }

  /* -------- Config -------- */

  void load_config()
  {
    YAML::Node cfg = YAML::LoadFile("src/mppi_planner/config/sim_config.yaml");

    dt_ = cfg["dt"].as<double>();
    obs_r_ = cfg["obs_r"].as<double>();
    obs_cmd_noise_std_dev_ = cfg["obs_cmd_noise"].as<double>();
    boundary_eps_ = cfg["boundary_eps"].as<double>();
    obs_v_min_ = cfg["obs_v_min"].as<double>();
    obs_v_max_ = cfg["obs_v_max"].as<double>();
    num_obs_ = cfg["num_obs"].as<int>();
    goalThreshold = cfg["goal_threshold"].as<double>();
    timeHorizon = cfg["time_horizon"].as<double>();
    timeHorizonObst = cfg["time_horizon_obst"].as<double>();
    maxNeighbors = cfg["max_neighbours"].as<double>();
    neighborDist = cfg["neighbor_dist"].as<double>();
    auto pose_lim = cfg["pose_lim"];
    pose_lim_.resize(2);
    for (int i = 0; i < 2; ++i)
    {
      pose_lim_[i].resize(2);
      pose_lim_[i][0] = pose_lim[i][0].as<double>();
      pose_lim_[i][1] = pose_lim[i][1].as<double>();
    }
  }
  private:
  /* -------- State -------- */

  struct State
  {
    double x{0}, y{0}, vx{0}, vy{0};
  };
  std::unordered_map<std::string, State> obs_states_;
  std::unordered_map<std::string,
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_vel_pubs_;
  std::unordered_map<std::string,
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  std::unordered_map<std::string,
    std::shared_ptr<GaussianNoiseSampler>> samplers_;

  rclcpp::TimerBase::SharedPtr timer_;

  /* -------- Parameters -------- */

  std::shared_ptr<RVO::RVOPlanner> mRVO{nullptr};
  std::vector<double> mLimitGoal;
  int num_obs_;
  double goalThreshold;
  double timeHorizon;
  double timeHorizonObst;
  double maxNeighbors;
  double neighborDist;
  double dt_;
  double obs_r_;
  double boundary_eps_;
  double obs_cmd_noise_std_dev_;
  double obs_v_min_;
  double obs_v_max_;

  double minX_, minY_, maxX_, maxY_;
  std::vector<std::vector<double>> pose_lim_;

  bool init_vel_set_{false};
};

/* ---------------- Main ---------------- */

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleObstacleDynamics>());
  rclcpp::shutdown();
  return 0;
}
