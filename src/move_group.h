#ifndef __CHOMP_GENERATOR_MOVE_GROUP_H
#define __CHOMP_GENERATOR_MOVE_GROUP_H

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf/transform_listener.h>
#include <moveit/move_group/move_group_capability.h>
#include <boost/algorithm/string/join.hpp>
#include <boost/tokenizer.hpp>
#include <moveit/macros/console_colors.h>
#include <moveit/move_group/node_name.h>

namespace move_group
{
  class MoveGroupExe
  {
    public:
      MoveGroupExe(const planning_scene_monitor::PlanningSceneMonitorPtr& psm, bool debug);
      ~MoveGroupExe();

      void status();

    private:
      void configureCapabilities();

      ros::NodeHandle node_handle_;
      MoveGroupContextPtr context_;
      boost::shared_ptr<pluginlib::ClassLoader<MoveGroupCapability> > capability_plugin_loader_;
      std::vector<boost::shared_ptr<MoveGroupCapability> > capabilities_;
  };
}

#endif // ifndef __CHOMP_GENERATOR_MOVE_GROUP_H
