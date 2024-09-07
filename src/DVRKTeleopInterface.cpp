#include "dvrk_teleop_interface/DVRKTeleopInterface.hpp"

namespace dvrk_teleop_interface {

DVRKTeleopInterface::DVRKTeleopInterface(any_node::Node::NodeHandlePtr nh)
    : any_node::Node(nh) {}

bool DVRKTeleopInterface::init() {
  // Load parameters
  publishRate_ = param<double>("publish_rate", 200);
  baseFrameId_ = param<std::string>("base_frame_id", "base");
  teleopLeftWrenchTopic_ =
      param<std::string>("teleop_left_wrench_topic", "/teleop/left/wrench");
  teleopRightWrenchTopic_ =
      param<std::string>("teleop_right_wrench_topic", "/teleop/right/wrench");
  teleopLeftGripperTopic_ =
      param<std::string>("teleop_gripper_left_topic", "/teleop/left/gripper");
 teleopRightGripperTopic_ =
      param<std::string>("teleop_gripper_right_topic", "/teleop/right/gripper");
  poseExpiration_ = param<double>("pose_expiration", 0.01);
  wrenchExpiration_ = param<double>("wrench_expiration", 0.1);
  leftGripperLimits_ = param<std::vector<double>>("left_gripper_limits", {0., 1.0});
  rightGripperLimits_ = param<std::vector<double>>("right_gripper_limits", {0., 1.0});
  forceScaling_ = param<double>("force_scaling", 0.2);
  maxForce_ = param<double>("max_force", 20.0);

  // Initialize subscribers
  dvrkPoseLeftSub_ = nh_->subscribe(
      "/MTML/measured_cp", 1, &DVRKTeleopInterface::dvrkPoseLeftCallback, this);
  dvrkPoseRightSub_ = nh_->subscribe(
      "/MTMR/measured_cp", 1, &DVRKTeleopInterface::dvrkPoseRightCallback, this);
  dvrkClutchSub_ = nh_->subscribe(
      "/mobile_manipulator_state_machine/clutch", 1, &DVRKTeleopInterface::dvrkClutchCallback, this);
  dvrkGripperLeftSub_ = nh_->subscribe(
      "/MTML/gripper/measured_js", 1, &DVRKTeleopInterface::dvrkGripperLeftCallback, this);
  dvrkGripperRightSub_ = nh_->subscribe(
      "/MTMR/gripper/measured_js", 1, &DVRKTeleopInterface::dvrkGripperRightCallback, this);
  teleopLeftWrenchSub_ = nh_->subscribe(
      teleopLeftWrenchTopic_, 1, &DVRKTeleopInterface::teleopLeftWrenchCallback, this);
  teleopRightWrenchSub_ = nh_->subscribe(
      teleopRightWrenchTopic_, 1, &DVRKTeleopInterface::teleopRightWrenchCallback, this);

  // Initialize publishers
  dvrkLeftWrenchPub_ =
      nh_->advertise<geometry_msgs::WrenchStamped>("/MTML/body/servo_cf", 1);
  dvrkRightWrenchPub_ =
      nh_->advertise<geometry_msgs::WrenchStamped>("/MTMR/body/servo_cf", 1);
  leftTeleopClutchPub_ = nh_->advertise<std_msgs::Bool>("/teleop/left/leader_clutch", 1);
  rightTeleopClutchPub_ = nh_->advertise<std_msgs::Bool>("/teleop/right/leader_clutch", 1);
  leftPoseDesPub_ = nh_->advertise<geometry_msgs::TransformStamped>(
      "/teleop/left/leader_pose", 1);
  rightPoseDesPub_ = nh_->advertise<geometry_msgs::TransformStamped>(
      "/teleop/right/leader_pose", 1);
  leftGripperPub_ = nh_->advertise<sensor_msgs::JointState>(teleopLeftGripperTopic_, 1);
  rightGripperPub_ = nh_->advertise<sensor_msgs::JointState>(teleopRightGripperTopic_, 1);
  

  // Initialize coord transform
  dvrkCoordToNormalCoord_ << 0, 1, 0, -1, 0, 0, 0, 0, 1;
  normalCoordToDvrkCoord_ << 0, 1, 0, -1, 0, 0, 0, 0, 1;

  any_worker::WorkerOptions workerOptions;
  workerOptions.name_ = ros::this_node::getName() + std::string{"_broadcast"};
  workerOptions.callback_ = boost::bind(&DVRKTeleopInterface::update, this, _1);
  workerOptions.timeStep_ = 1.0 / publishRate_;
  workerOptions.defaultPriority_ = 0;

  if (!addWorker(workerOptions)) {
    ROS_ERROR_STREAM("Could not add worker: " << workerOptions.name_);
    return false;
  }

  return true;
}

void DVRKTeleopInterface::cleanup() {}

bool DVRKTeleopInterface::update(const any_worker::WorkerEvent &event) {
  if ((ros::Time::now() - dvrkPoseLeft_.header.stamp).toSec() < poseExpiration_) {
    processDVRKPose(dvrkPoseLeft_, leftPoseDesPub_);
  }

  if ((ros::Time::now() - dvrkPoseRight_.header.stamp).toSec() < poseExpiration_) {
    processDVRKPose(dvrkPoseRight_, rightPoseDesPub_);
  }

  std_msgs::Bool teleopClutch;
  teleopClutch.data= dvrkClutch_.data;
  leftTeleopClutchPub_.publish(teleopClutch);
  rightTeleopClutchPub_.publish(teleopClutch);

  if ((ros::Time::now() - lastLeftWrenchTime_).toSec() < wrenchExpiration_) {
    processTeleopWrench(teleopLeftWrench_, dvrkPoseLeft_, dvrkLeftWrenchPub_);
  }

  if ((ros::Time::now() - lastRightWrenchTime_).toSec() < wrenchExpiration_) {
    processTeleopWrench(teleopRightWrench_, dvrkPoseRight_, dvrkRightWrenchPub_);
  }

  if (dvrkGripperLeft_.position.size() >= 1){
    leftGripperPub_.publish(processGripperLimits(dvrkGripperLeft_, leftGripperLimits_));
  }

  if (dvrkGripperRight_.position.size() >= 1){
    rightGripperPub_.publish(processGripperLimits(dvrkGripperRight_, rightGripperLimits_));
  }

  return true;
}

void DVRKTeleopInterface::processDVRKPose(const geometry_msgs::TransformStamped &dvrk_pose,
                                   ros::Publisher &pub) {
  geometry_msgs::TransformStamped teleopPose = dvrk_pose;
  teleopPose.header.frame_id = baseFrameId_;
  Eigen::Vector3d trans;
  trans << teleopPose.transform.translation.x,
      teleopPose.transform.translation.y,
      teleopPose.transform.translation.z;
  trans = dvrkCoordToNormalCoord_ * trans;
  teleopPose.transform.translation.x = trans[0];
  teleopPose.transform.translation.y = trans[1];
  teleopPose.transform.translation.z = trans[2];
  Eigen::Vector3d rot;
  rot << teleopPose.transform.rotation.x,
      teleopPose.transform.rotation.y, teleopPose.transform.rotation.z;
  rot = dvrkCoordToNormalCoord_ * rot;
  // yes its a quaternion but because its a coord system transform this works
  teleopPose.transform.rotation.x = rot[0];
  teleopPose.transform.rotation.y = rot[1];
  teleopPose.transform.rotation.z = rot[2];
  pub.publish(teleopPose);
}

void DVRKTeleopInterface::processTeleopWrench(const geometry_msgs::WrenchStamped &wrench,
                                        const geometry_msgs::TransformStamped &dvrk_pose,
                                        ros::Publisher &pub) {
  geometry_msgs::WrenchStamped dvrkWrench = wrench;
  dvrkWrench.header.stamp = ros::Time::now();
  Eigen::Vector3d force;
  force << dvrkWrench.wrench.force.x, dvrkWrench.wrench.force.y,
      dvrkWrench.wrench.force.z;
  Eigen::Quaterniond quat;
  quat.w() = dvrk_pose.transform.rotation.w;
  quat.x() = dvrk_pose.transform.rotation.x;
  quat.y() = dvrk_pose.transform.rotation.y;
  quat.z() = dvrk_pose.transform.rotation.z;
  Eigen::Matrix3d frameCorrection = quat.inverse().toRotationMatrix();

  force = forceScaling_ * frameCorrection * normalCoordToDvrkCoord_ * force;

  if (force.norm() > maxForce_) {
    force = maxForce_ * force / force.norm();
  }

  dvrkWrench.wrench.force.x = force[0];
  dvrkWrench.wrench.force.y = force[1];
  dvrkWrench.wrench.force.z = force[2];
  dvrkWrench.wrench.torque.x = 0.0;
  dvrkWrench.wrench.torque.y = 0.0;
  dvrkWrench.wrench.torque.z = 0.0;
  pub.publish(dvrkWrench);
}

void DVRKTeleopInterface::dvrkPoseLeftCallback(
    const geometry_msgs::TransformStamped::ConstPtr &msg) {
  dvrkPoseLeft_ = *msg;
}

void DVRKTeleopInterface::dvrkPoseRightCallback(
    const geometry_msgs::TransformStamped::ConstPtr &msg) {
  dvrkPoseRight_ = *msg;
}

void DVRKTeleopInterface::dvrkClutchCallback(
    const std_msgs::Bool::ConstPtr &msg) {
  dvrkClutch_ = *msg;
}

void DVRKTeleopInterface::teleopLeftWrenchCallback(
    const geometry_msgs::WrenchStamped::ConstPtr &msg) {
  teleopLeftWrench_ = *msg;
  lastLeftWrenchTime_ = ros::Time::now();
}

void DVRKTeleopInterface::teleopRightWrenchCallback(
    const geometry_msgs::WrenchStamped::ConstPtr &msg) {
  teleopRightWrench_ = *msg;
  lastRightWrenchTime_ = ros::Time::now();
}

void DVRKTeleopInterface::dvrkGripperLeftCallback(const sensor_msgs::JointState::ConstPtr &msg) {
  dvrkGripperLeft_ = *msg;
}

void DVRKTeleopInterface::dvrkGripperRightCallback(const sensor_msgs::JointState::ConstPtr &msg) {
  dvrkGripperRight_ = *msg;
}

sensor_msgs::JointState DVRKTeleopInterface::processGripperLimits(const sensor_msgs::JointState& gripperState, std::vector<double> gripperLimits){
  sensor_msgs::JointState newGripperState = gripperState;
  newGripperState.position[0] = (newGripperState.position[0] - gripperLimits[0])/(gripperLimits[1] - gripperLimits[0]);
  newGripperState.position[0] = std::clamp(newGripperState.position[0], 0.0, 1.0);
  return newGripperState;
}

} /* namespace dvrk_teleop_interface */
