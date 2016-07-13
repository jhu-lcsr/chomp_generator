
#include <iostream>
#include <Eigen/Dense>

#include <ros/ros.h>

#include <chomp_generator/stomp_generator.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include "move_group.h"

#include <visualization_msgs/Marker.h>

using namespace chomp_generator;

using namespace move_group;
using namespace planning_pipeline;
using namespace planning_scene;
using namespace planning_scene_monitor;

using namespace stomp_moveit::cost_functions;

StompGenerator::StompGenerator() :
  nh_("~")
{
  monitor_.reset(new PlanningSceneMonitor("robot_description"));
  move_group_.reset(new MoveGroupExe(monitor_, false));

  costfunction_.reset(new stomp_moveit::cost_functions::TrajectoryAvoidance());

  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/stomp_traj_lines", 5, true);
}

void StompGenerator::generate_paths(int n) {
  // Get hooks
  MoveGroupContextPtr ctx = move_group_->getContext();
  PlanningPipelinePtr pipeline = ctx->planning_pipeline_;
  PlanningScenePtr scene = monitor_->getPlanningScene();

  // Reset stored paths
  costfunction_->clear_trajectories();

  // Compute IK
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(scene->getCurrentState()));
  Eigen::Affine3d ee_goal;
  ee_goal.setIdentity();
  ee_goal.translate(Eigen::Vector3d(0.5,0.0,0.5));
  //ee_goal = kinematic_state->getGlobalLinkTransform("right/hand");

  bool ik_success = kinematic_state->setFromIK(pipeline->getRobotModel()->getJointModelGroup("arm"), ee_goal, 5, 0.1);
  if(ik_success) {
    ROS_INFO_STREAM("Computed IK to: "<<ee_goal.translation());
  } else {
    ROS_ERROR_STREAM("Could not compute IK to: "<<ee_goal.translation());
    return;
  }

  // Create planning request
  planning_interface::MotionPlanRequest req;
  req.group_name = "arm";
  req.allowed_planning_time = 10.0;
  {
    moveit_msgs::Constraints gc;
    gc.name = "target";
    // Start State

    // Goal
    if(0){ // Cart gaol
      moveit_msgs::PositionConstraint pc;
      pc.header.frame_id = "base_link";
      pc.header.stamp = ros::Time::now();
      {
        shape_msgs::SolidPrimitive region;
        region.type = shape_msgs::SolidPrimitive::SPHERE;
        region.dimensions.push_back(0.01);
        pc.constraint_region.primitives.push_back(region);
      }
      {
        geometry_msgs::Pose pose;
        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = 0.5;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        pc.constraint_region.primitive_poses.push_back(pose);
      }
      gc.position_constraints.push_back(pc);
    }
    if(1){ // Joint goal
      auto joint_models = monitor_->getRobotModel()->getJointModels();
      for(auto joint : joint_models) {
        if(joint->getType() == moveit::core::JointModel::REVOLUTE || joint->getType() == moveit::core::JointModel::PRISMATIC) {
          moveit_msgs::JointConstraint jc;
          jc.joint_name = joint->getName();
          jc.position = kinematic_state->getJointPositions(joint->getName())[0];
          jc.tolerance_below = 0.05;
          jc.tolerance_above = 0.05;
          jc.weight = 1.0;
          gc.joint_constraints.push_back(jc);

          req.start_state.joint_state.name.push_back(joint->getName());
          req.start_state.joint_state.position.push_back(scene->getCurrentState().getJointPositions(joint->getName())[0]);
        }
      }
    }
    req.goal_constraints.push_back(gc);
  }

  // Planning response
  planning_interface::MotionPlanResponse res;

  visualization_msgs::MarkerArray traj_marker_array;
  for(int i=0; i<n; i++) {

    bool success = pipeline->generatePlan(
        monitor_->getPlanningScene(),
        req,
        res);

    ROS_INFO_STREAM("Plan "<<i<<(success ? " SUCCEEDED" : " FAILED"));
    if(success) {
      // Create introspection marker
      visualization_msgs::Marker traj_marker;
      traj_marker.header.stamp = ros::Time::now();
      traj_marker.header.frame_id = "right/shoulder_base";
      traj_marker.ns = "arm";
      traj_marker.id = i;
      traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
      traj_marker.scale.x = 0.01;
      traj_marker.color.r = 1.0;
      traj_marker.color.a = 1.0;

      // Add the result to the costmap
      std::vector<Eigen::Affine3d> ee_traj;
      for(size_t p=0; p < res.trajectory_->getWayPointCount(); p++) {
        Eigen::Affine3d t = res.trajectory_->getWayPoint(p).getFrameTransform("right/hand");
        ee_traj.push_back(t);

        geometry_msgs::Point traj_point;
        traj_point.x = t.translation()(0);
        traj_point.y = t.translation()(1);
        traj_point.z = t.translation()(2);
        traj_marker.points.push_back(traj_point);
      }
      TrajectoryAvoidance::add_trajectory(ee_traj);
      traj_marker_array.markers.push_back(traj_marker);
    }
  }

  // Publishthe paths
  marker_pub_.publish(traj_marker_array);
}

