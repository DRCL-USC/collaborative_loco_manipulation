#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <ocs2_ros_interfaces/mrt/DummyObserver.h>

#include "ocs2_object_manipulation/definitions.h"

namespace ocs2 {
namespace object_manipulation {

class ObjectDummyVisualization : public DummyObserver {
 public:
  explicit ObjectDummyVisualization(ros::NodeHandle& nodeHandle) { launchVisualizerNode(nodeHandle); }

  ~ObjectDummyVisualization() override = default;

  void update(const SystemObservation& observation, const PrimalSolution& policy, const CommandData& command) override;

 private:
  void launchVisualizerNode(ros::NodeHandle& nodeHandle);

  ros::Publisher jointPublisher_;
};

}  // namespace object_manipulation
}  // namespace ocs2
