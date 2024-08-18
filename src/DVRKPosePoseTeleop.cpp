#include "dvrk_pose_pose_teleop/DVRKPosePoseTeleop.hpp"

namespace dvrk_pose_pose_teleop {

DVRKPosePoseTeleop::DVRKPosePoseTeleop(any_node::Node::NodeHandlePtr nh) : any_node::Node(nh) {}

bool DVRKPosePoseTeleop::init() {
  // Load parameters
  publishRate_ = param<double>("publish_rate", 200);
  baseFrameId_ = param<std::string>("base_frame_id", "base");
  dvrkLeftPoseTopic_ = param<std::string>("dvrk_left_pose_topic", "/MTML/measured_cp");
  dvrkRightPoseTopic_ = param<std::string>("dvrk_right_pose_topic", "/MTMR/measured_cp");
  dvrkClutchTopic_ = param<std::string>("dvrk_clutch_topic", "/DVRK/clutch");
  teleopWrenchTopic_ = param<std::string>("teleop_wrench_topic", "/teleop_wrench");
  poseExpiration_ = param<double>("pose_expiration", 0.01);
  wrenchExpiration_ = param<double>("wrench_expiration", 0.1);

  // Initialize subscribers
  dvrkPoseLeftSub_ = nh_->subscribe(dvrkLeftPoseTopic_, 1, &DVRKPosePoseTeleop::dvrkPoseLeftCallback, this);
  dvrkPoseRightSub_ = nh_->subscribe(dvrkRightPoseTopic_, 1, &DVRKPosePoseTeleop::dvrkPoseRightCallback, this);
  dvrkClutchSub_ = nh_->subscribe(dvrkClutchTopic_, 1, &DVRKPosePoseTeleop::dvrkClutchCallback, this);
  teleopWrenchSub_ = nh_->subscribe(teleopWrenchTopic_, 1, &DVRKPosePoseTeleop::teleopWrenchCallback, this);

  // Initialize publishers
  dvrkWrenchPub_ = nh_->advertise<geometry_msgs::WrenchStamped>("/DVRK/wrench", 1);
  teleopClutchPub_ = nh_->advertise<sensor_msgs::Joy>("/quest/joystick", 1);

  any_worker::WorkerOptions workerOptions;
  workerOptions.name_ = ros::this_node::getName() + std::string{"_broadcast"};
  workerOptions.callback_ = boost::bind(&DVRKPosePoseTeleop::update, this, _1);
  workerOptions.timeStep_ = 1.0 / publishRate_;
  workerOptions.defaultPriority_ = 0;

  if (!addWorker(workerOptions)) {
    ROS_ERROR_STREAM("Could not add worker: " << workerOptions.name_);
    return false;
  }

  return true;
}

void DVRKPosePoseTeleop::cleanup() {
}

bool DVRKPosePoseTeleop::update(const any_worker::WorkerEvent& event) {
  geometry_msgs::TransformStamped teleopLeft = dvrkPoseLeft_;
  geometry_msgs::TransformStamped teleopRight = dvrkPoseRight_;

  if ((ros::Time::now() - teleopLeft.header.stamp).toSec() < poseExpiration_) {
    teleopLeft.header.frame_id = baseFrameId_;
    teleopLeft.child_frame_id = "hand_left";
    Eigen::Vector3d leftTrans;
    leftTrans << teleopLeft.transform.translation.x, teleopLeft.transform.translation.y, teleopLeft.transform.translation.z;
    leftTrans = dvrkCoordToNormalCoord_ * leftTrans;
    teleopLeft.transform.translation.x = leftTrans[0];
    teleopLeft.transform.translation.y = leftTrans[1];
    teleopLeft.transform.translation.z = leftTrans[2];
    tfBroadcaster_.sendTransform(teleopLeft);
  }

  if ((ros::Time::now() - teleopRight.header.stamp).toSec() < poseExpiration_) {
    teleopRight.header.frame_id = baseFrameId_;
    teleopRight.child_frame_id = "hand_right";
    Eigen::Vector3d rightTrans;
    rightTrans << teleopRight.transform.translation.x, teleopRight.transform.translation.y, teleopRight.transform.translation.z;
    rightTrans = dvrkCoordToNormalCoord_ * rightTrans;
    teleopRight.transform.translation.x = rightTrans[0];
    teleopRight.transform.translation.y = rightTrans[1];
    teleopRight.transform.translation.z = rightTrans[2];
    tfBroadcaster_.sendTransform(teleopRight);
  }

  sensor_msgs::Joy teleopClutch;
  teleopClutch.header.stamp = ros::Time::now();
  teleopClutch.buttons.resize(12);
  teleopClutch.buttons[JoystickButtons::LeftClutch] = dvrkClutch_.data;
  teleopClutch.buttons[JoystickButtons::RightClutch] = dvrkClutch_.data;
  teleopClutchPub_.publish(teleopClutch);

  if ((ros::Time::now() - lastWrenchTime_).toSec() < wrenchExpiration_) {
    geometry_msgs::WrenchStamped dvrkWrench = teleopWrench_;
    dvrkWrench.header.stamp = ros::Time::now();
    Eigen::Vector3d force;
    force << dvrkWrench.wrench.force.x, dvrkWrench.wrench.force.y, dvrkWrench.wrench.force.z;
    force = dvrkCoordToNormalCoord_.inverse() * force;
    dvrkWrench.wrench.force.x = force[0];
    dvrkWrench.wrench.force.y = force[1];
    dvrkWrench.wrench.force.z = force[2];
    dvrkWrench.wrench.torque.x = 0.0;
    dvrkWrench.wrench.torque.y = 0.0;
    dvrkWrench.wrench.torque.z = 0.0;
    dvrkWrenchPub_.publish(dvrkWrench);
  }
  return true;
}

void DVRKPosePoseTeleop::dvrkPoseLeftCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  dvrkPoseLeft_ = *msg;
}

void DVRKPosePoseTeleop::dvrkPoseRightCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
  dvrkPoseRight_ = *msg;
}

void DVRKPosePoseTeleop::dvrkClutchCallback(const std_msgs::Bool::ConstPtr& msg) {
  dvrkClutch_ = *msg;
}

void DVRKPosePoseTeleop::teleopWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  teleopWrench_ = *msg;
  lastWrenchTime_ = ros::Time::now();
}

} /* namespace dvrk_pose_pose_teleop */
