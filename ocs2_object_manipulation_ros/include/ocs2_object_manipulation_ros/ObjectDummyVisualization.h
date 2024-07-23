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

      explicit ObjectDummyVisualization(ros::NodeHandle &nodeHandle, const std::string taskfile);

      ~ObjectDummyVisualization() override = default;

      void update(const SystemObservation &observation, const PrimalSolution &policy, const CommandData &command) override;

    private:
      void launchVisualizerNode(ros::NodeHandle &nodeHandle);
      void publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t &mpcTimeTrajectory,
                                           const vector_array_t &mpcStateTrajectory);
      void publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories &targetTrajectories);
      visualization_msgs::Marker ObjectTrajectory(ros::Time timeStamp, const SystemObservation &observation);
      visualization_msgs::Marker ObjectTarget(ros::Time timeStamp, const CommandData &command);
      visualization_msgs::MarkerArray wrench(ros::Time timeStamp, const SystemObservation &observation);
      visualization_msgs::MarkerArray obstacles(ros::Time timeStamp);

      ros::Publisher desiredBasePositionPublisher_;
      ros::Publisher stateOptimizedPublisher_;
      ros::Publisher wrenchPublisher_;
      ros::Publisher objectPublisher_;
      ros::Publisher obstaclesPublisher_;
      const std::string taskFile_;
      scalar_array_t init_yaw;
      scalar_t F_max;
      std::vector<std::pair<scalar_t, scalar_t>> obstacles_pose;
    };

  } // namespace object_manipulation
} // namespace ocs2
