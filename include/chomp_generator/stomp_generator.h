#ifndef __CHOMP_GENERATOR_STOMP_GENERATOR_H
#define __CHOMP_GENERATOR_STOMP_GENERATOR_H

#include <ros/ros.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <chomp_generator/trajectory_avoidance.h>

/**

PlanningSceneMonitor < robot_description
PlanningPipeline pieline < PlanningSceneMonitor

PlannerManaer < PlanningPipeline

for 1 to n:
  pipeline.generatePlan()
end

 **/

namespace move_group {
  class MoveGroupExe;
}

namespace chomp_generator {

  class StompGenerator {

    public:
      StompGenerator();
      ~StompGenerator() {}

      void generate_paths(int n);

    protected:
      ros::NodeHandle nh_;

      // Stomp Costfunction
      std::shared_ptr<stomp_moveit::cost_functions::TrajectoryAvoidance> costfunction_;

      // MoveIt structures
      planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
      std::shared_ptr<move_group::MoveGroupExe> move_group_;

  };
}

#endif // ifndef __CHOMP_GENERATOR_STOMP_GENERATOR_H
