///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2016, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/* 
  Author: Sam Pfeiffer
  Node to get out of collision state heavily based on
  https://github.com/ros-planning/moveit_ros/blob/indigo-devel/planning/planning_request_adapter_plugins/src/fix_start_state_collision.cpp
  from Ioan Sucan
*/
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>

class FixStartStateCollision
{
public:

  static const std::string DT_PARAM_NAME;
  static const std::string JIGGLE_PARAM_NAME;
  static const std::string ATTEMPTS_PARAM_NAME;
  static const std::string GROUP_NAME;

  FixStartStateCollision() :
      nh_("~")
    , robot_model_loader_(new robot_model_loader::RobotModelLoader("robot_description"))
    , planning_scene_monitor_(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader_))
  {
    if (!nh_.getParam(DT_PARAM_NAME, time_for_goal_))
    {
      time_for_goal_ = 2.0;
      ROS_INFO_STREAM("Param '" << DT_PARAM_NAME << "' was not set. Using default value: " << time_for_goal_);
    }
    else
      ROS_INFO_STREAM("Param '" << DT_PARAM_NAME << "' was set to " << time_for_goal_);

    if (!nh_.getParam(JIGGLE_PARAM_NAME, jiggle_fraction_))
    {
      jiggle_fraction_ = 0.02;
      ROS_INFO_STREAM("Param '" << JIGGLE_PARAM_NAME << "' was not set. Using default value: " << jiggle_fraction_);
    }
    else
      ROS_INFO_STREAM("Param '" << JIGGLE_PARAM_NAME << "' was set to " << jiggle_fraction_);

    if (!nh_.getParam(ATTEMPTS_PARAM_NAME, sampling_attempts_))
    {
      sampling_attempts_ = 100;
      ROS_INFO_STREAM("Param '" << ATTEMPTS_PARAM_NAME << "' was not set. Using default value: " << sampling_attempts_);
    }
    else
    {
      if (sampling_attempts_ < 1)
      {
        sampling_attempts_ = 1;
        ROS_WARN_STREAM("Param '" << ATTEMPTS_PARAM_NAME << "' needs to be at least 1.");
      }
      ROS_INFO_STREAM("Param '" << ATTEMPTS_PARAM_NAME << "' was set to " << sampling_attempts_);
    }
    if (!nh_.getParam(GROUP_NAME, group_name_))
    {
      group_name_ = "arm_torso";
      ROS_INFO_STREAM("Param '" << GROUP_NAME << "' was not set. Using default value: " << group_name_);
    }
    else
      ROS_INFO_STREAM("Param '" << GROUP_NAME << "' was set to " << group_name_);

    if(planning_scene_monitor_->getPlanningScene())
    {
      ROS_DEBUG_STREAM("Starting context monitors...");
      planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
      planning_scene_monitor_->startWorldGeometryMonitor();
      planning_scene_monitor_->startStateMonitor();
      ROS_INFO_STREAM("Context monitors started for " << nh_.getNamespace());
    }
    else
    {
      ROS_ERROR_STREAM("Planning scene not configured for " << nh_.getNamespace());
    }
    robot_traj_.reset(new robot_trajectory::RobotTrajectory(robot_model_loader_->getModel(), ""));
    execute_srv_ = nh_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path");
    get_out_of_collision_srv_ = nh_.advertiseService("/get_out_of_collision", &FixStartStateCollision::getOutOfCollision, this);

  }

  bool getOutOfCollision(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res)
  {
    // get the start state
    robot_state::RobotState start_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();

    collision_detection::CollisionRequest creq;
    creq.group_name = group_name_;
    collision_detection::CollisionResult cres;
    planning_scene_monitor_->getPlanningScene()->checkCollision(creq, cres, start_state);
    if (cres.collision)
    {
      // Rerun in verbose mode
      collision_detection::CollisionRequest vcreq = creq;
      collision_detection::CollisionResult vcres;
      vcreq.verbose = true;
      planning_scene_monitor_->getPlanningScene()->checkCollision(vcreq, vcres, start_state);

      if (creq.group_name.empty())
        ROS_INFO("Start state appears to be in collision");
      else
        ROS_INFO_STREAM("Start state appears to be in collision with respect to group " << creq.group_name);

      robot_state::RobotStatePtr prefix_state(new robot_state::RobotState(start_state));
      random_numbers::RandomNumberGenerator &rng = prefix_state->getRandomNumberGenerator();

      const std::vector<const robot_model::JointModel*> &jmodels =
              planning_scene_monitor_->getPlanningScene()->getRobotModel()->getJointModelGroup(group_name_)->getJointModels();
      // tries to use wheels and stuff...
      //planning_scene_monitor_->getPlanningScene()->getRobotModel()->getJointModels();

      bool found = false;
      for (int c = 0 ; !found && c < sampling_attempts_ ; ++c)
      {
        for (std::size_t i = 0 ; !found && i < jmodels.size() ; ++i)
        {
          std::vector<double> sampled_variable_values(jmodels[i]->getVariableCount());
          const double *original_values = prefix_state->getJointPositions(jmodels[i]);
          jmodels[i]->getVariableRandomPositionsNearBy(rng, &sampled_variable_values[0], original_values, jmodels[i]->getMaximumExtent() * jiggle_fraction_);
          start_state.setJointPositions(jmodels[i], sampled_variable_values);
          collision_detection::CollisionResult cres;
          planning_scene_monitor_->getPlanningScene()->checkCollision(creq, cres, start_state);
          if (!cres.collision)
          {
            found = true;
            ROS_INFO("Found a valid state near the start state at distance %lf after %d attempts", prefix_state->distance(start_state), c);
          }
        }
      }

      if (found)
      {
        moveit_msgs::ExecuteKnownTrajectory ekt;
        trajectory_msgs::JointTrajectory trajectory;
        robot_state::RobotState initial_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
        robot_traj_->setRobotTrajectoryMsg(initial_state, trajectory);
        robot_traj_->setGroupName(group_name_);
        robot_traj_->addSuffixWayPoint(start_state, time_for_goal_);
        moveit_msgs::RobotTrajectory robot_traj_msg;
        robot_traj_->getRobotTrajectoryMsg(robot_traj_msg);
        ekt.request.trajectory = robot_traj_msg;
        ekt.request.wait_for_execution = true;
        if(execute_srv_.call(ekt)){
            ROS_INFO_STREAM("Succesfully went out of collision");
        }
        else
        {
            ROS_ERROR_STREAM("Failed calling execute known path...");
        }
        return true;
      }
      else
      {
        ROS_WARN("Unable to find a valid state nearby the start state (using jiggle fraction of %lf and %u sampling attempts). Passing the original planning request to the planner.",
                 jiggle_fraction_, sampling_attempts_);
        return true;
      }
    }
    else
    {
      if (creq.group_name.empty())
        ROS_INFO_STREAM("Start state is valid");
      else
        ROS_INFO_STREAM("Start state is valid with respect to group " << creq.group_name);
      return true;
    }
  }

private:

  ros::NodeHandle nh_;
  double time_for_goal_;
  double jiggle_fraction_;
  int sampling_attempts_;
  std::string group_name_;
  const robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  robot_trajectory::RobotTrajectoryPtr robot_traj_;
  ros::ServiceServer get_out_of_collision_srv_;
  ros::ServiceClient execute_srv_;
};

const std::string FixStartStateCollision::DT_PARAM_NAME = "time_for_goal";
const std::string FixStartStateCollision::JIGGLE_PARAM_NAME = "jiggle_fraction";
const std::string FixStartStateCollision::ATTEMPTS_PARAM_NAME = "max_sampling_attempts";
const std::string FixStartStateCollision::GROUP_NAME = "group_name";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_out_of_collision");
  ros::NodeHandle nh;

  FixStartStateCollision fssc;
  ros::spin();

  return 0;
}
