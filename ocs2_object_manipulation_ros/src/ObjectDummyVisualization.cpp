#include "ocs2_object_manipulation_ros/ObjectDummyVisualization.h"

namespace ocs2 {
namespace object_manipulation {

void ObjectDummyVisualization::update(const SystemObservation& observation, const PrimalSolution& policy, const CommandData& command) {
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(3);
  joint_state.position.resize(3);
  joint_state.name[0] = "x";
  joint_state.name[1] = "y";
  joint_state.name[2] = "theta";
  joint_state.position[0] = observation.state(0);
  joint_state.position[1] = observation.state(1);
  joint_state.position[2] = observation.state(2);
  jointPublisher_.publish(joint_state);
}

void ObjectDummyVisualization::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
  jointPublisher_ = nodeHandle.advertise<sensor_msgs::JointState>("joint_states", 1);
}

}  // namespace object_manipulation
}  // namespace ocs2
