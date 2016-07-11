
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf/transform_listener.h>
#include <moveit/move_group/move_group_capability.h>
#include <boost/algorithm/string/join.hpp>
#include <boost/tokenizer.hpp>
#include <moveit/macros/console_colors.h>
#include <moveit/move_group/node_name.h>

#include "move_group.h"

static const std::string ROBOT_DESCRIPTION = "robot_description";    // name of the robot description (a param name, so it can be changed externally)

using namespace move_group;

MoveGroupExe::MoveGroupExe(const planning_scene_monitor::PlanningSceneMonitorPtr& psm, bool debug) :
  node_handle_("~")
{
  // if the user wants to be able to disable execution of paths, they can just set this ROS param to false
  bool allow_trajectory_execution;
  node_handle_.param("allow_trajectory_execution", allow_trajectory_execution, true);

  context_.reset(new MoveGroupContext(psm, allow_trajectory_execution, debug));

  // start the capabilities
  configureCapabilities();
}

MoveGroupExe::~MoveGroupExe()
{
  capabilities_.clear();
  context_.reset();
  capability_plugin_loader_.reset();
}

void MoveGroupExe::status()
{
  if (context_)
  {
    if (context_->status())
    {
      if (capabilities_.empty())
        printf(MOVEIT_CONSOLE_COLOR_BLUE "\nAll is well but no capabilities are loaded. There will be no party :(\n\n" MOVEIT_CONSOLE_COLOR_RESET);
      else
        printf(MOVEIT_CONSOLE_COLOR_GREEN "\nAll is well! Everyone is happy! You can start planning now!\n\n" MOVEIT_CONSOLE_COLOR_RESET);
      fflush(stdout);
    }
  }
  else
    ROS_ERROR("No MoveGroup context created. Nothing will work.");
}

void MoveGroupExe::configureCapabilities()
{
  try
  {
    capability_plugin_loader_.reset(new pluginlib::ClassLoader<MoveGroupCapability>("moveit_ros_move_group", "move_group::MoveGroupCapability"));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating plugin loader for move_group capabilities: " << ex.what());
    return;
  }

  // add individual capabilities move_group supports
  std::string capability_plugins;
  if (node_handle_.getParam("capabilities", capability_plugins))
  {
    boost::char_separator<char> sep(" ");
    boost::tokenizer<boost::char_separator<char> > tok(capability_plugins, sep);
    for(boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin() ; beg != tok.end(); ++beg)
    {
      std::string plugin = *beg;
      try
      {
        printf(MOVEIT_CONSOLE_COLOR_CYAN "Loading '%s'...\n" MOVEIT_CONSOLE_COLOR_RESET, plugin.c_str());
        MoveGroupCapability *cap = capability_plugin_loader_->createUnmanagedInstance(plugin);
        cap->setContext(context_);
        cap->initialize();
        capabilities_.push_back(boost::shared_ptr<MoveGroupCapability>(cap));
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_ERROR_STREAM("Exception while loading move_group capability '" << plugin << "': " << ex.what() << std::endl
            << "Available capabilities: " << boost::algorithm::join(capability_plugin_loader_->getDeclaredClasses(), ", "));
      }
    }
  }
  std::stringstream ss;
  ss << std::endl;
  ss << std::endl;
  ss << "********************************************************" << std::endl;
  ss << "* MoveGroup using: " << std::endl;
  for (std::size_t i = 0 ; i < capabilities_.size() ; ++i)
    ss << "*     - " << capabilities_[i]->getName() << std::endl;
  ss << "********************************************************" << std::endl;
  ROS_INFO_STREAM(ss.str());
}

