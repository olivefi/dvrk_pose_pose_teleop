#include <ros/ros.h>
#include <any_node/any_node.hpp>
#include <dvrk_pose_pose_teleop/DVRKPosePoseTeleop.hpp>


int main(int argc, char **argv)
{
  any_node::Nodewrap<dvrk_pose_pose_teleop::DVRKPosePoseTeleop> node(argc, argv, "dvrk_pose_pose_teleop", 1);
  return static_cast<int>(!node.execute());
}


