#pragma once

// ros
#include <ros/ros.h>

#include <any_node/any_node.hpp>
#include <tf2_ros/transform_broadcaster.h>


namespace dvrk_pose_pose_teleop {

//! This currently only re-publishes the robot state to two separate topics: pose and joint state.
class DVRKPosePoseTeleop : public any_node::Node {
 public:
  DVRKPosePoseTeleop(any_node::Node::NodeHandlePtr nh);
  ~DVRKPosePoseTeleop() override = default;

  bool init() override;
  void cleanup() override;
  bool update(const any_worker::WorkerEvent& event);

 protected:
  std::string baseFrameId_;
  std::string odomFrameId_;

  geometry_msgs::TransformStamped baseTransform_;

 private:
  ros::Time prevCallTime_;};

} /* namespace dvrk_pose_pose_teleop */
