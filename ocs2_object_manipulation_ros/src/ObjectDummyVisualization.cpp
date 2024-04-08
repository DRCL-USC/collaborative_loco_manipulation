#include "ocs2_object_manipulation_ros/ObjectDummyVisualization.h"

namespace ocs2 {
namespace object_manipulation {

void ObjectDummyVisualization::update(const SystemObservation& observation, const PrimalSolution& policy, const CommandData& command) {
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.name[0] = "slider_to_cart";
  joint_state.name[1] = "cart_to_pole";
  joint_state.position[0] = observation.state(1);
  joint_state.position[1] = observation.state(0);
  jointPublisher_.publish(joint_state);
}

void ObjectDummyVisualization::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
  jointPublisher_ = nodeHandle.advertise<sensor_msgs::JointState>("joint_states", 1);
}

}  // namespace object_manipulation
}  // namespace ocs2
