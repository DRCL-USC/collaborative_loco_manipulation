#include <memory>

#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_object_manipulation/ObjectInterface.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/SolverObserverRosCallbacks.h>

int main(int argc, char** argv) {
  const std::string robotName = "object";

  // task file
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName(programArgs[1]);

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mpc");
  ros::NodeHandle nodeHandle;

  // Robot interface
  const std::string taskFile = ros::package::getPath("ocs2_object_manipulation") + "/config/" + taskFileFolderName + "/task.info";
  const std::string libFolder = ros::package::getPath("ocs2_object_manipulation") + "/auto_generated";
  ocs2::object_manipulation::ObjectInterface objectInterface(taskFile, libFolder, true /*verbose*/);

  // MPC
  ocs2::GaussNewtonDDP_MPC mpc(objectInterface.mpcSettings(), objectInterface.ddpSettings(), objectInterface.getRollout(),
                               objectInterface.getOptimalControlProblem(), objectInterface.getInitializer());

  // observer for the input limits constraints
  auto createStateInputBoundsObserver = [&]() {
    const std::string observingLagrangianTerm = "InputLimits";
    const ocs2::scalar_array_t observingTimePoints{0.0, 0.5};
    std::vector<std::string> metricsTopicNames;
    std::vector<std::string> multiplierTopicNames;
    for (const auto& t : observingTimePoints) {
      const int timeMs = static_cast<int>(t * 1000.0);
      metricsTopicNames.push_back("metrics/" + observingLagrangianTerm + "/" + std::to_string(timeMs) + "MsLookAhead");
      multiplierTopicNames.push_back("multipliers/" + observingLagrangianTerm + "/" + std::to_string(timeMs) + "MsLookAhead");
    }
    auto lagrangianCallback = ocs2::ros::createLagrangianCallback(nodeHandle, observingTimePoints, metricsTopicNames,
                                                                  ocs2::ros::CallbackInterpolationStrategy::linear_interpolation);
    auto multiplierCallback = ocs2::ros::createMultiplierCallback(nodeHandle, observingTimePoints, multiplierTopicNames,
                                                                  ocs2::ros::CallbackInterpolationStrategy::linear_interpolation);
    return ocs2::SolverObserver::LagrangianTermObserver(ocs2::SolverObserver::Type::Intermediate, observingLagrangianTerm,
                                                        std::move(lagrangianCallback), std::move(multiplierCallback));
  };
  mpc.getSolverPtr()->addSolverObserver(createStateInputBoundsObserver());

  // Launch MPC ROS node
  ocs2::MPC_ROS_Interface mpcNode(mpc, robotName);
  mpcNode.launchNodes(nodeHandle);

  return 0;
}
