
#include <ros/ros.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <chomp_generator/chomp_generator.h>
#include <boost/shared_ptr.hpp>

using namespace chomp;

int main(int argc, char** argv) {

  ros::init(argc, argv, "genpaths");

  // Construct Planner
  moveit::core::RobotModelConstPtr robot_model;
  boost::shared_ptr<ChompPlanner> planner(new ChompPlanner(robot_model));

  // Construct planning request
  planning_scene::PlanningSceneConstPtr planning_scene;
  ChompParameters params;

  moveit_msgs::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  // Generate paths
  bool success = planner->solve(planning_scene, req, params, res);

  // Sample path in octomap
  // TODO: Update octomap

  // Publish it to rviz
  // TODO: Publish path 15
  // TODO: Publish octomap 15

  // Wait
  ros::spin();

  return 0;
}
