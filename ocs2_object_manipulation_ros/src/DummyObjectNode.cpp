#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ocs2_object_manipulation/ObjectInterface.h>
#include <ocs2_object_manipulation/definitions.h>

#include "ocs2_object_manipulation_ros/ObjectDummyVisualization.h"

int main(int argc, char** argv) {
  const std::string robotName = "object";

  // task file
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(programArgs[1]);

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nodeHandle;

  // Robot interface
  const std::string taskFile = ros::package::getPath("ocs2_object_manipulation") + "/config/" + taskFileFolderName + "/task.info";
  const std::string libFolder = ros::package::getPath("ocs2_object_manipulation") + "/auto_generated";
  ocs2::object_manipulation::ObjectInterface objectInterface(taskFile, libFolder, false /*verbose*/);

  // MRT
  ocs2::MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&objectInterface.getRollout());
  mrt.launchNodes(nodeHandle);

  // Visualization
  auto objectDummyVisualization = std::make_shared<ocs2::object_manipulation::ObjectDummyVisualization>(nodeHandle, taskFile);

  // Dummy loop
  ocs2::MRT_ROS_Dummy_Loop dummyObject(mrt, objectInterface.mpcSettings().mrtDesiredFrequency_,
                                         objectInterface.mpcSettings().mpcDesiredFrequency_);
  dummyObject.subscribeObservers({objectDummyVisualization});

  // initial state
  ocs2::SystemObservation initObservation;
  initObservation.state = objectInterface.getInitialState();
  initObservation.input.setZero(ocs2::object_manipulation::INPUT_DIM);
  initObservation.time = 0.0;

  // initial command
  const ocs2::TargetTrajectories initTargetTrajectories({0.0}, {objectInterface.getInitialTarget()},
                                                        {ocs2::vector_t::Zero(ocs2::object_manipulation::INPUT_DIM)});

  // Run dummy (loops while ros is ok)
  dummyObject.run(initObservation, initTargetTrajectories);

  return 0;
}
