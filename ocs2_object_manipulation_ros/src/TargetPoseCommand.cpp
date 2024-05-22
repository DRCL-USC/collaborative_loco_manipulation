#include <ocs2_object_manipulation/definitions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>
#include <ros/package.h>

using namespace ocs2;
using namespace object_manipulation;

namespace
{
  scalar_t targetDisplacementVelocity;
  scalar_t targetRotationVelocity;
} // namespace

scalar_t estimateTimeToTarget(const vector_t &desiredBaseDisplacement)
{
  const scalar_t &dx = desiredBaseDisplacement(0);
  const scalar_t &dy = desiredBaseDisplacement(1);
  const scalar_t &dyaw = desiredBaseDisplacement(2);
  const scalar_t rotationTime = std::abs(dyaw) / targetRotationVelocity;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / targetDisplacementVelocity;
  return std::max(rotationTime, displacementTime);
}

/**
 * Converts command line to TargetTrajectories.
 * @param [in] commadLineTarget : [deltaX, deltaY, deltaYaw]
 * @param [in] observation : the current observation
 */
TargetTrajectories commandLineToTargetTrajectories(const vector_t &commadLineTarget, const SystemObservation &observation)
{
  const vector_t currentPose = observation.state.segment<3>(0);
  const vector_t targetPose = [&]()
  {
    vector_t target(3);
    // p_x, p_y, and theta_z  are relative to current state
    target(0) = currentPose(0) + commadLineTarget(0);
    target(1) = currentPose(1) + commadLineTarget(1);
    target(2) = currentPose(2) + commadLineTarget(2) * M_PI / 180.0;
    return target;
  }();

  // target reaching duration
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);

  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << currentPose, vector_t::Zero(3);
  stateTrajectory[1] << targetPose, vector_t::Zero(3);

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

int main(int argc, char *argv[])
{
  const std::string robotName = "object";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;
  // Get node parameters
  const std::string taskFile = ros::package::getPath("ocs2_object_manipulation") + "/config/mpc/task.info";

  loadData::loadCppDataType(taskFile, "targetRotationVelocity", targetRotationVelocity);
  loadData::loadCppDataType(taskFile, "targetDisplacementVelocity", targetDisplacementVelocity);

  // goalPose: [deltaX, deltaY, deltaYaw]
  const scalar_array_t relativeBaseLimit{10.0, 10.0, 180.0};
  TargetTrajectoriesKeyboardPublisher targetPoseCommand(nodeHandle, robotName, relativeBaseLimit, &commandLineToTargetTrajectories);

  const std::string commandMsg = "Enter XY and Yaw (deg) displacements for the object, separated by spaces";
  targetPoseCommand.publishKeyboardCommand(commandMsg);

  // Successful exit
  return 0;
}
