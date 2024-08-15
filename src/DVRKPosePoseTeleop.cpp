#include "dvrk_pose_pose_teleop/DVRKPosePoseTeleop.hpp"

namespace dvrk_pose_pose_teleop {

DVRKPosePoseTeleop::DVRKPosePoseTeleop(any_node::Node::NodeHandlePtr nh) : any_node::Node(nh) {}

bool DVRKPosePoseTeleop::init() {
  const double publishFrequency = param<double>("publish_frequency", 100);

  baseFrameId_ = param<std::string>("base_frame_id", "base");
  odomFrameId_ = param<std::string>("odom_frame_id", "odom");

  baseTransform_.header.frame_id = odomFrameId_;
  baseTransform_.child_frame_id = baseFrameId_;
  prevCallTime_ = ros::Time::now();

  // Initialize robot state publisher
  std::string urdfName = param<std::string>("robot_description", "");

  // Update worker
  any_worker::WorkerOptions workerOptions;
  workerOptions.name_ = ros::this_node::getName() + std::string{"_broadcast"};
  workerOptions.callback_ = boost::bind(&DVRKPosePoseTeleop::update, this, _1);
  workerOptions.timeStep_ = 1.0 / publishFrequency;
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
  return true;
}

} /* namespace dvrk_pose_pose_teleop */
