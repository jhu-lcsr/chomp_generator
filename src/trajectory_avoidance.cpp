/**
 * @file trajectory_avoidance.cpp
 * @brief This defines a cost function for trajectory avoidance.
 *
 * @author Jorge Nicho
 * @date March 30, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @license Software License Agreement (Apache License)\n
 * \n
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at\n
 * \n
 * http://www.apache.org/licenses/LICENSE-2.0\n
 * \n
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <moveit/robot_state/conversions.h>
#include "chomp_generator/trajectory_avoidance.h"

PLUGINLIB_EXPORT_CLASS(stomp_moveit::cost_functions::TrajectoryAvoidance,stomp_moveit::cost_functions::StompCostFunction)

const std::string DEFAULT_COLLISION_DETECTOR = "IndustrialFCL";

namespace stomp_moveit
{
namespace cost_functions
{

std::vector<std::vector<Eigen::Affine3d> > TrajectoryAvoidance::accepted_trajectories;

TrajectoryAvoidance::TrajectoryAvoidance():
    name_("TrajectoryAvoidancePlugin"),
    nh_(),
    robot_state_(),
    collision_clearance_(0.0),
    collision_penalty_(0.0)
{
  // TODO Auto-generated constructor stub

  // TODO Create trajectory subscriber
}

TrajectoryAvoidance::~TrajectoryAvoidance()
{
  // TODO Auto-generated destructor stub
}

bool TrajectoryAvoidance::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,XmlRpc::XmlRpcValue& config)
{
  robot_model_ptr_ = robot_model_ptr;
  group_name_ = group_name;
  tip_link_id_ = "right/hand";
  return configure(config);
}

bool TrajectoryAvoidance::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp_core::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace moveit::core;

  planning_scene_ = planning_scene;
  plan_request_ = req;
  error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  // storing robot state
  robot_state_.reset(new RobotState(robot_model_ptr_));
  if(!robotStateMsgToRobotState(req.start_state,*robot_state_,true))
  {
    ROS_ERROR("%s Failed to get current robot state from request",getName().c_str());
    return false;
  }

  return true;
}

bool TrajectoryAvoidance::computeCosts(const Eigen::MatrixXd& parameters,
                          std::size_t start_timestep,
                          std::size_t num_timesteps,
                          int iteration_number,
                          int rollout_number,
                          Eigen::VectorXd& costs,
                          bool& validity)
{
  using namespace moveit::core;

  //ROS_INFO_STREAM("Trajectories: "<<accepted_trajectories.size());

  if(!robot_state_)
  {
    ROS_ERROR("%s Robot State has not been updated",getName().c_str());
    return false;
  }

  // initializing result array
  costs = Eigen::VectorXd::Zero(num_timesteps);

  // robot state
  const JointModelGroup* joint_group = robot_model_ptr_->getJointModelGroup(group_name_);
  std::vector<double> joint_values(parameters.rows(),0);

  if(parameters.cols()<start_timestep + num_timesteps)
  {
    ROS_ERROR_STREAM("Size in the 'parameters' matrix is less than required");
    return false;
  }

  // Compute the cost for each point in the new trajectory
  for (auto t=start_timestep; t<start_timestep + num_timesteps; ++t)
  {
    // Initialize costs for this step
    costs(t) = 0.0;

    if(accepted_trajectories.size() == 0) {
      continue;
    }

    // Get the joint position from this timestep
    Eigen::VectorXd joint_position = parameters.col(t);

    // Set the robot state
    robot_state_->setJointGroupPositions(joint_group,joint_position);
    robot_state_->update();

    // Get the cartesian position for the point in the new trajectory
    const Eigen::Affine3d &pose = robot_state_->getFrameTransform(tip_link_id_);

    // For each existing trajectory
    int traj_index = 0;
    for(auto &trajectory : accepted_trajectories) {

      // Find the closest point in the existing trajectory
      float min_distance = 1.0; //[m] //MAX_DISTANCE;

      int p=0;
      for(auto &point : trajectory) {
        if(p > 0.1*trajectory.size() && p < 0.9*trajectory.size()) {
          float distance = (point.translation() - pose.translation()).norm();
          if(distance < min_distance) {
            min_distance = distance;
          }
        }
        p++;
      }

      // Add the cost for this trajectory to this point
      float cost = 1.0 / std::max(min_distance*min_distance, 0.0001f);
      std::cerr<<"Traj "<<traj_index<<" point ("<<t<<") cost: "<<cost<<std::endl;
      costs(t) += cost;
      traj_index++;
    }
  }

  // scaling cost
  double max = costs.maxCoeff();
  costs /= (max > 1e-8) ? max : 1;

  return true;
}

bool TrajectoryAvoidance::configure(const XmlRpc::XmlRpcValue& config)
{
  // check parameter presence
  auto members = {"collision_clearance","collision_penalty","cost_weight"};
  for(auto& m : members)
  {
    if(!config.hasMember(m))
    {
      ROS_ERROR("%s failed to find one or more required parameters",getName().c_str());
      return false;
    }
  }

  try
  {
    XmlRpc::XmlRpcValue c = config;
    collision_clearance_ = static_cast<double>(c["collision_clearance"]);
    collision_penalty_ = static_cast<double>(c["collision_penalty"]);
    cost_weight_ = static_cast<double>(c["cost_weight"]);
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to parse configuration parameters",name_.c_str());
    return false;
  }

  return true;
}

void TrajectoryAvoidance::done(bool success,int total_iterations,double final_cost)
{
  robot_state_.reset();
}

void TrajectoryAvoidance::clear_trajectories()
{
  accepted_trajectories.clear();
}

void TrajectoryAvoidance::add_trajectory(std::vector<Eigen::Affine3d> &trajectory)
{
  accepted_trajectories.push_back(trajectory);
}

} /* namespace cost_functions */
} /* namespace stomp_moveit */
