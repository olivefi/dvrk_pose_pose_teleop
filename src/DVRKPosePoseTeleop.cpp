#include "dvrk_pose_pose_teleop/DVRKPosePoseTeleop.hpp"

namespace dvrk_pose_pose_teleop {

DVRKPosePoseTeleop::DVRKPosePoseTeleop(any_node::Node::NodeHandlePtr nh)
    : any_node::Node(nh) {}

bool DVRKPosePoseTeleop::init() {
  // Load parameters
  publishRate_ = param<double>("publish_rate", 200);
  baseFrameId_ = param<std::string>("base_frame_id", "base");
  dvrkLeftPoseTopic_ =
      param<std::string>("dvrk_left_pose_topic", "/MTML/measured_cp");
  dvrkRightPoseTopic_ =
      param<std::string>("dvrk_right_pose_topic", "/MTMR/measured_cp");
  dvrkClutchTopic_ = param<std::string>(
      "dvrk_clutch_topic", "/mobile_manipulator_state_machine/clutch");
  teleopLeftWrenchTopic_ =
      param<std::string>("teleop_left_wrench_topic", "/teleop/left/wrench");
  teleopRightWrenchTopic_ =
      param<std::string>("teleop_right_wrench_topic", "/teleop/right/wrench");
  poseExpiration_ = param<double>("pose_expiration", 0.01);
  wrenchExpiration_ = param<double>("wrench_expiration", 0.1);

  // Initialize subscribers
  dvrkPoseLeftSub_ = nh_->subscribe(
      dvrkLeftPoseTopic_, 1, &DVRKPosePoseTeleop::dvrkPoseLeftCallback, this);
  dvrkPoseRightSub_ = nh_->subscribe(
      dvrkRightPoseTopic_, 1, &DVRKPosePoseTeleop::dvrkPoseRightCallback, this);
  dvrkClutchSub_ = nh_->subscribe(
      dvrkClutchTopic_, 1, &DVRKPosePoseTeleop::dvrkClutchCallback, this);
  teleopLeftWrenchSub_ = nh_->subscribe(
      teleopLeftWrenchTopic_, 1, &DVRKPosePoseTeleop::teleopLeftWrenchCallback, this);
  teleopRightWrenchSub_ = nh_->subscribe(
      teleopRightWrenchTopic_, 1, &DVRKPosePoseTeleop::teleopRightWrenchCallback, this);

  // Initialize publishersCall
  dvrkLeftWrenchPub_ =
      nh_->advertise<geometry_msgs::WrenchStamped>("/MTML/body/servo_cf", 1);
  dvrkRightWrenchPub_ =
      nh_->advertise<geometry_msgs::WrenchStamped>("/MTMR/body/servo_cf", 1);
  teleopClutchPub_ = nh_->advertise<sensor_msgs::Joy>("/quest/joystick", 1);
  leftPoseDesPub_ = nh_->advertise<geometry_msgs::TransformStamped>(
      "/teleop/left/leader_pose", 1);
  rightPoseDesPub_ = nh_->advertise<geometry_msgs::TransformStamped>(
      "/teleop/right/leader_pose", 1);

  // Initialize coord transform
  dvrkCoordToNormalCoord_ << 0, 1, 0, -1, 0, 0, 0, 0, 1;
  normalCoordToDvrkCoord_ << 0, 1, 0, -1, 0, 0, 0, 0, 1;

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

void DVRKPosePoseTeleop::cleanup() {}

bool DVRKPosePoseTeleop::update(const any_worker::WorkerEvent &event) {
  geometry_msgs::TransformStamped teleopLeft = dvrkPoseLeft_;
  geometry_msgs::TransformStamped teleopRight = dvrkPoseRight_;

  if ((ros::Time::now() - teleopLeft.header.stamp).toSec() < poseExpiration_) {
    teleopLeft.header.frame_id = baseFrameId_;
    teleopLeft.child_frame_id = "hand_left";
    Eigen::Vector3d leftTrans;
    leftTrans << teleopLeft.transform.translation.x,
        teleopLeft.transform.translation.y, teleopLeft.transform.translation.z;
    leftTrans = dvrkCoordToNormalCoord_ * leftTrans;
    teleopLeft.transform.translation.x = leftTrans[0];
    teleopLeft.transform.translation.y = leftTrans[1];
    teleopLeft.transform.translation.z = leftTrans[2];
    // yes its a quaternion but because its a coord system transform this works
    Eigen::Vector3d leftRot;
    leftRot << teleopLeft.transform.rotation.x, teleopLeft.transform.rotation.y,
        teleopLeft.transform.rotation.z;
    leftRot = dvrkCoordToNormalCoord_ * leftRot;
    teleopLeft.transform.rotation.x = leftRot[0];
    teleopLeft.transform.rotation.y = leftRot[1];
    teleopLeft.transform.rotation.z = leftRot[2];
    leftPoseDesPub_.publish(teleopLeft);
    // tfBroadcaster_.sendTransform(teleopLeft);
  }

  if ((ros::Time::now() - teleopRight.header.stamp).toSec() < poseExpiration_) {
    teleopRight.header.frame_id = baseFrameId_;
    teleopRight.child_frame_id = "hand_right";
    Eigen::Vector3d rightTrans;
    rightTrans << teleopRight.transform.translation.x,
        teleopRight.transform.translation.y,
        teleopRight.transform.translation.z;
    rightTrans = dvrkCoordToNormalCoord_ * rightTrans;
    teleopRight.transform.translation.x = rightTrans[0];
    teleopRight.transform.translation.y = rightTrans[1];
    teleopRight.transform.translation.z = rightTrans[2];
    Eigen::Vector3d rightRot;
    rightRot << teleopRight.transform.rotation.x,
        teleopRight.transform.rotation.y, teleopRight.transform.rotation.z;
    rightRot = dvrkCoordToNormalCoord_ * rightRot;
    teleopRight.transform.rotation.x = rightRot[0];
    teleopRight.transform.rotation.y = rightRot[1];
    teleopRight.transform.rotation.z = rightRot[2];
    rightPoseDesPub_.publish(teleopRight);
    // tfBroadcaster_.sendTransform(teleopRight);
  }

  sensor_msgs::Joy teleopClutch;
  teleopClutch.header.stamp = ros::Time::now();
  teleopClutch.buttons.resize(12);
  teleopClutch.buttons[JoystickButtons::LeftClutch] = dvrkClutch_.data;
  teleopClutch.buttons[JoystickButtons::RightClutch] = dvrkClutch_.data;
  teleopClutchPub_.publish(teleopClutch);

  if ((ros::Time::now() - lastLeftWrenchTime_).toSec() < wrenchExpiration_) {
    geometry_msgs::WrenchStamped dvrkWrench = teleopLeftWrench_;
    dvrkWrench.header.stamp = ros::Time::now();
    Eigen::Vector3d force;
    force << dvrkWrench.wrench.force.x, dvrkWrench.wrench.force.y,
        dvrkWrench.wrench.force.z;
    force = 0.2 * normalCoordToDvrkCoord_ * force;
    dvrkWrench.wrench.force.x = force[0];
    dvrkWrench.wrench.force.y = force[1];
    dvrkWrench.wrench.force.z = force[2];
    dvrkWrench.wrench.torque.x = 0.0;
    dvrkWrench.wrench.torque.y = 0.0;
    dvrkWrench.wrench.torque.z = 0.0;
    dvrkLeftWrenchPub_.publish(dvrkWrench);
  }

  if ((ros::Time::now() - lastRightWrenchTime_).toSec() < wrenchExpiration_) {
    geometry_msgs::WrenchStamped dvrkWrench = teleopRightWrench_;
    dvrkWrench.header.stamp = ros::Time::now();
    Eigen::Vector3d force;
    force << dvrkWrench.wrench.force.x, dvrkWrench.wrench.force.y,
        dvrkWrench.wrench.force.z;
    force = 0.2 * normalCoordToDvrkCoord_ * force;
    dvrkWrench.wrench.force.x = force[0];
    dvrkWrench.wrench.force.y = force[1];
    dvrkWrench.wrench.force.z = force[2];
    dvrkWrench.wrench.torque.x = 0.0;
    dvrkWrench.wrench.torque.y = 0.0;
    dvrkWrench.wrench.torque.z = 0.0;
    dvrkRightWrenchPub_.publish(dvrkWrench);
  }
  return true;
}

void DVRKPosePoseTeleop::dvrkPoseLeftCallback(
    const geometry_msgs::TransformStamped::ConstPtr &msg) {
  dvrkPoseLeft_ = *msg;
}

void DVRKPosePoseTeleop::dvrkPoseRightCallback(
    const geometry_msgs::TransformStamped::ConstPtr &msg) {
  dvrkPoseRight_ = *msg;
}

void DVRKPosePoseTeleop::dvrkClutchCallback(
    const std_msgs::Bool::ConstPtr &msg) {
  dvrkClutch_ = *msg;
}

void DVRKPosePoseTeleop::teleopLeftWrenchCallback(
    const geometry_msgs::WrenchStamped::ConstPtr &msg) {
  teleopLeftWrench_ = *msg;
  lastLeftWrenchTime_ = ros::Time::now();
}

void DVRKPosePoseTeleop::teleopRightWrenchCallback(
    const geometry_msgs::WrenchStamped::ConstPtr &msg) {
  teleopRightWrench_ = *msg;
  lastRightWrenchTime_ = ros::Time::now();
}

} /* namespace dvrk_pose_pose_teleop */
