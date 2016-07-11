
#include <ros/ros.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <chomp_generator/chomp_generator.h>
#include <boost/shared_ptr.hpp>

#include <chomp_generator/stomp_generator.h>

using namespace chomp_generator;

int main(int argc, char** argv) {

  ros::init(argc, argv, "genpaths");

  ros::NodeHandle nh;

  auto sg = std::make_shared<StompGenerator>();

  //sg->generate_paths(1);

  ros::spin();

  return 0;
}
