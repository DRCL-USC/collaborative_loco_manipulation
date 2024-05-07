#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <ocs2_ros_interfaces/mrt/DummyObserver.h>

#include "ocs2_object_manipulation/definitions.h"
#include <visualization_msgs/MarkerArray.h>

namespace ocs2
{
  namespace object_manipulation
  {

    class ObjectDummyVisualization : public DummyObserver
    {
    public:
      /** Visualization settings (publicly available) */
      std::string frameId_ = "world";                                                                 // Frame name all messages are published in
      scalar_t trajectoryLineWidth_ = 0.01;                                                          // LineThickness for trajectories

      explicit ObjectDummyVisualization(ros::NodeHandle &nodeHandle) { launchVisualizerNode(nodeHandle); }

      ~ObjectDummyVisualization() override = default;

      void update(const SystemObservation &observation, const PrimalSolution &policy, const CommandData &command) override;

    private:
      void launchVisualizerNode(ros::NodeHandle &nodeHandle);
      void publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t &mpcTimeTrajectory,
                                           const vector_array_t &mpcStateTrajectory, const ModeSchedule &modeSchedule);
      void publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories &targetTrajectories);
      visualization_msgs::Marker ObjectTrajectory(const SystemObservation &observation);
      visualization_msgs::Marker ObjectTarget(const CommandData &command);

      ros::Publisher desiredBasePositionPublisher_;
      ros::Publisher stateOptimizedPublisher_;
      ros::Publisher vis_pub;
    };

  } // namespace object_manipulation
} // namespace ocs2
