
#include <chomp_generator/stomp_generator.h>

#include "move_group.h"

using namespace chomp_generator;

using namespace move_group;
using namespace planning_pipeline;
using namespace planning_scene_monitor;

using namespace stomp_moveit::cost_functions;

StompGenerator::StompGenerator() :
  nh_("~")
{
  monitor_.reset(new PlanningSceneMonitor("robot_description"));
  move_group_.reset(new MoveGroupExe(monitor_, false));

  costfunction_.reset(new stomp_moveit::cost_functions::TrajectoryAvoidance());

  std::vector<Eigen::Affine3d> t(10);

  TrajectoryAvoidance::add_trajectory(t);
}

void StompGenerator::generate_paths(int n) {

  // Reset stored paths
  // TODO: try to just set static variable in costmap, and have it printed out

  // FOR 1..n
  // TODO pipeline.generatePlan()
  // Store path

  // Render paths
}

