#pragma once

#include <algorithm>
#include <Eigen/Core>
#include <ros/ros.h>


#include <any_node/any_node.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>

namespace dvrk_teleop_interface {

enum JoystickButtons { LeftClutch = 10, RightClutch = 11 };
enum ControlStates { Arms = 0, Legs = 1};

class DVRKTeleopInterface : public any_node::Node {
public:
  DVRKTeleopInterface(any_node::Node::NodeHandlePtr nh);
  ~DVRKTeleopInterface() override = default;

  bool init() override;
  void cleanup() override;
  bool update(const any_worker::WorkerEvent &event);

protected:
  void processTeleopWrench(const geometry_msgs::WrenchStamped &wrench, const geometry_msgs::TransformStamped &dvrk_pose,
                     ros::Publisher &pub);
  void processDVRKPoseForArms(const geometry_msgs::TransformStamped &dvrk_pose, ros::Publisher &pub);

  void processDVRKPoseForLegs(const geometry_msgs::TransformStamped &dvrk_pose, ros::Publisher &pub);

  ControlStates controlState_ = Arms;

  tf2_ros::TransformBroadcaster tfBroadcaster_;
  Eigen::Matrix3d dvrkCoordToNormalCoord_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d normalCoordToDvrkCoord_ = Eigen::Matrix3d::Identity();

  bool twistDesInitialized_ = false;
  geometry_msgs::TransformStamped twistDesInitPose_;

  // Externally settable variables
  std::string baseFrameId_;
  std::string teleopLeftWrenchTopic_;
  std::string teleopRightWrenchTopic_;
  std::string teleopLeftGripperTopic_;
  std::string teleopRightGripperTopic_;
  double publishRate_;
  double poseExpiration_;
  double wrenchExpiration_;
  double forceScaling_;
  double maxForce_;
  std::vector<double> leftGripperLimits_;
  std::vector<double> rightGripperLimits_;

  // Stuff we receive from DVRK
  ros::Subscriber dvrkPoseLeftSub_;
  ros::Subscriber dvrkPoseRightSub_;
  ros::Subscriber dvrkClutchSub_;
  ros::Subscriber dvrkGripperLeftSub_;
  ros::Subscriber dvrkGripperRightSub_;
  ros::Subscriber dvrkControlStateSub_;

  void dvrkPoseLeftCallback(const geometry_msgs::TransformStamped::ConstPtr &msg);
  void dvrkPoseRightCallback(const geometry_msgs::TransformStamped::ConstPtr &msg);
  void dvrkClutchCallback(const std_msgs::Bool::ConstPtr &msg);
  void dvrkGripperLeftCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void dvrkGripperRightCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void dvrkControlStateCallback(const std_msgs::Empty::ConstPtr &msg);

  sensor_msgs::JointState processGripperLimits(const sensor_msgs::JointState& gripperState, std::vector<double> gripperLimits);

  geometry_msgs::TransformStamped dvrkPoseLeft_;
  geometry_msgs::TransformStamped dvrkPoseRight_;
  std_msgs::Bool dvrkClutch_;
  sensor_msgs::JointState dvrkGripperLeft_;
  sensor_msgs::JointState dvrkGripperRight_;

  // Values we receive from teleop follower device
  ros::Subscriber teleopLeftWrenchSub_;
  ros::Subscriber teleopRightWrenchSub_;

  void teleopLeftWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
  void teleopRightWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);

  geometry_msgs::WrenchStamped teleopLeftWrench_;
  geometry_msgs::WrenchStamped teleopRightWrench_;
  ros::Time lastLeftWrenchTime_;
  ros::Time lastRightWrenchTime_;

  // Stuff we (re-)publish
  ros::Publisher dvrkLeftWrenchPub_;
  ros::Publisher dvrkRightWrenchPub_;
  ros::Publisher leftTeleopClutchPub_;
  ros::Publisher rightTeleopClutchPub_;
  ros::Publisher leftPoseDesPub_;
  ros::Publisher rightPoseDesPub_;
  ros::Publisher leftGripperPub_;
  ros::Publisher rightGripperPub_;
  ros::Publisher twistDesPub_;

  Eigen::Quaterniond rosQuatToEigen(const geometry_msgs::Quaternion &rosQuat);
};
} /* namespace dvrk_teleop_interface */
