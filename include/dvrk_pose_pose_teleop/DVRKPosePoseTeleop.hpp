#pragma once

#include <ros/ros.h>
#include <Eigen/Core>

#include <any_node/any_node.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

namespace dvrk_pose_pose_teleop {

enum JoystickButtons { LeftClutch = 10, RightClutch = 11 };

class DVRKPosePoseTeleop : public any_node::Node {
 public:
  DVRKPosePoseTeleop(any_node::Node::NodeHandlePtr nh);
  ~DVRKPosePoseTeleop() override = default;

  bool init() override;
  void cleanup() override;
  bool update(const any_worker::WorkerEvent& event);

 protected:
  tf2_ros::TransformBroadcaster tfBroadcaster_;
  Eigen::Matrix3d dvrkCoordToNormalCoord_ = Eigen::Matrix3d::Identity();

  // Externally settable variables
  std::string baseFrameId_;
  std::string dvrkLeftPoseTopic_;
  std::string dvrkRightPoseTopic_;
  std::string dvrkClutchTopic_;
  std::string teleopWrenchTopic_;
  double publishRate_;
  double poseExpiration_;
  double wrenchExpiration_;

  // Stuff we receive from DVRK
  ros::Subscriber dvrkPoseLeftSub_;
  ros::Subscriber dvrkPoseRightSub_;
  ros::Subscriber dvrkClutchSub_;

  void dvrkPoseLeftCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
  void dvrkPoseRightCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
  void dvrkClutchCallback(const std_msgs::Bool::ConstPtr& msg);

  geometry_msgs::TransformStamped dvrkPoseLeft_;
  geometry_msgs::TransformStamped dvrkPoseRight_;
  std_msgs::Bool dvrkClutch_;

  // Values we receive from teleop follower device
  ros::Subscriber teleopWrenchSub_;

  void teleopWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

  geometry_msgs::WrenchStamped teleopWrench_;
  ros::Time lastWrenchTime_;

  // Stuff we (re-)publish
  ros::Publisher dvrkWrenchPub_;
  ros::Publisher teleopClutchPub_;
};
} /* namespace dvrk_pose_pose_teleop */
