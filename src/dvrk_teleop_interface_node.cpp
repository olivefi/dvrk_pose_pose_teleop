#include <ros/ros.h>
#include <any_node/any_node.hpp>
#include <dvrk_teleop_interface/DVRKTeleopInterface.hpp>


int main(int argc, char **argv)
{
  any_node::Nodewrap<dvrk_teleop_interface::DVRKTeleopInterface> node(argc, argv, "dvrk_teleop_interface", 1);
  return static_cast<int>(!node.execute());
}


