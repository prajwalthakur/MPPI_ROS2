#ifndef NAV_ROV_H_
#define NAV_ROV_H_

#include "Agent.h"
#include "KdTree.h"
#include "Definitions.h"
#include "Obstacle.h"
// #include "gazebo_msgs/ModelStates.h"
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "RVOSimulator.h"
#include <string>
#include <random>

//Uniform Sampler 
class UniformSampler
{
public:
  UniformSampler(double minVal, double maxVal, int seed)
  : gen_(seed), dist_(minVal, maxVal) {}

  double sample()
  {
    return dist_(gen_);
  }
private:
  std::mt19937 gen_;
  std::uniform_real_distribution<double> dist_;
};

namespace RVO {

    class Agent;
    class Obstacle;
    class KdTree;

    class RVOPlanner{
    public:
        RVOPlanner(std::string simulation);

        void setupScenario(float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, float goalThreshold);
        void setupScenario(float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const std::vector<double>& limitGoal, float goalThreshold, float randGoalChangeThreshold );
        //void updateState_gazebo(gazebo_msgs::ModelStates::ConstPtr model_msg, std::string agent_name);
        void addAgentinSim(nav_msgs::msg::Odometry::SharedPtr msg, std::string agent_name);
        void UpdateAgentStateSim(nav_msgs::msg::Odometry::SharedPtr msg, std::string agent_name);
        bool ifAgentExistInmap(std::string name);
        void setGoal();
        void setGoalByAgent(std::string agentName, const std::vector<double>& limits, const std::string &model);
        void randGoal(const float limit_goal[4], const std::string &model="default");
        void randomOnceGoal(const float limit_goal[4]);
        bool arrived();
        bool isAgentArrived(const std::string agentName);
        void setGoal(std::vector<geometry_msgs::msg::Point> set_goals);
        void setInitial();
        void setPreferredVelocities();
        void setPreferredVelocitiesbyNameMap();
        void setPreferredVelocitiesbyName(std::string agentName, RVO::Vector2 noise);
        std::vector<RVO::Vector2*>  step();
        std::unordered_map<std::string, std::shared_ptr<RVO::Vector2>> stepCenteralised();
        float mGoalThreshold = 0.03;
        float mRandThresholdPercentage = 50;
        
        
    private:

        RVO::RVOSimulator* sim;
        std::string simulator;
        std::vector <RVO::Vector2> goals;
        std::unordered_map<std::string, RVO::Vector2> mAgentGoalMap;
        std::vector<std::string> mAgentNameCollection;
        bool IfInitial = false;
        std::vector<RVO::Vector2 *> newVelocities;
        std::shared_ptr<UniformSampler>  mXCoordSampler{nullptr};
        std::shared_ptr<UniformSampler>  mYCoordSampler{nullptr};
        std::shared_ptr<UniformSampler>  mRandomDecisionSampler{nullptr};

        friend class Agent;
        friend class KdTree;
        friend class Obstacle;
    };
}

#endif