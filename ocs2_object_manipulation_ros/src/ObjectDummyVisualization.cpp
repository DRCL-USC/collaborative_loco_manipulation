#include "ocs2_object_manipulation_ros/ObjectDummyVisualization.h"
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2
{
  namespace object_manipulation
  {

    void ObjectDummyVisualization::update(const SystemObservation &observation, const PrimalSolution &policy, const CommandData &command)
    {

      const auto timeStamp = ros::Time::now();

      // Publish object trajectory
      visualization_msgs::MarkerArray objectmarkers;
      visualization_msgs::Marker target_marker = ObjectTarget(timeStamp, command);
      visualization_msgs::Marker object_marker = ObjectTrajectory(timeStamp, observation);
      objectmarkers.markers.push_back(target_marker);
      objectmarkers.markers.push_back(object_marker);
      objectPublisher_.publish(objectmarkers);

      // Publish wrenches
      visualization_msgs::MarkerArray wrench_markers = wrench(timeStamp, observation);
      wrenchPublisher_.publish(wrench_markers);

      // Publish obstacles
      visualization_msgs::MarkerArray obstacle_markers = obstacles(timeStamp);
      obstaclesPublisher_.publish(obstacle_markers);

      // Publish desired trajectory
      publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_);
      publishOptimizedStateTrajectory(timeStamp, policy.timeTrajectory_, policy.stateTrajectory_, policy.modeSchedule_);
    }

    void ObjectDummyVisualization::launchVisualizerNode(ros::NodeHandle &nodeHandle)
    {
      desiredBasePositionPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("/desiredTrajectory", 1);
      stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("/optimizedStateTrajectory", 1);
      objectPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/object_markers", 0);
      wrenchPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/wrench_markers", 0);
      obstaclesPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/obstacle_markers", 0);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void ObjectDummyVisualization::publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories &targetTrajectories)
    {
      const auto &stateTrajectory = targetTrajectories.stateTrajectory;

      // Reserve com messages
      std::vector<geometry_msgs::Point> desiredBasePositionMsg;
      desiredBasePositionMsg.reserve(stateTrajectory.size());

      // std::cout << "stateTrajectory.size(): " << stateTrajectory.size() << std::endl;

      for (size_t j = 0; j < stateTrajectory.size(); j++)
      {
        const auto state = stateTrajectory.at(j);

        // Construct base pose msg
        geometry_msgs::Pose pose;
        vector_t basePose(3);
        basePose << state(0), state(1), 0.25;
        pose.position = getPointMsg(basePose);

        // Fill message containers
        desiredBasePositionMsg.push_back(pose.position);
      }

      // std::cout << "desiredBasePositionMsg.size(): " << desiredBasePositionMsg.size() << std::endl;

      // Headers
      auto comLineMsg = getLineMsg(std::move(desiredBasePositionMsg), Color::green, trajectoryLineWidth_);
      comLineMsg.header = getHeaderMsg(frameId_, timeStamp);
      comLineMsg.id = 0;

      // Publish
      desiredBasePositionPublisher_.publish(comLineMsg);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void ObjectDummyVisualization::publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t &mpcTimeTrajectory,
                                                                   const vector_array_t &mpcStateTrajectory, const ModeSchedule &modeSchedule)
    {
      if (mpcTimeTrajectory.empty() || mpcStateTrajectory.empty())
      {
        return; // Nothing to publish
      }

      // Reserve Com Msg
      std::vector<geometry_msgs::Point> mpcComPositionMsgs;
      mpcComPositionMsgs.reserve(mpcStateTrajectory.size());

      // Extract Com and Feet from state
      std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t &state)
                    {
        // Fill com position and pose msgs
        geometry_msgs::Pose pose;
        vector_t basePose(3);
        basePose << state(0), state(1), 0.25;
        pose.position = getPointMsg(basePose);
        mpcComPositionMsgs.push_back(pose.position); });

      // Add headers and Id
      auto comLineMsg = getLineMsg(std::move(mpcComPositionMsgs), Color::red, trajectoryLineWidth_);
      comLineMsg.header = getHeaderMsg(frameId_, timeStamp);
      comLineMsg.id = 0;

      stateOptimizedPublisher_.publish(comLineMsg);
    }

    visualization_msgs::Marker ObjectDummyVisualization::ObjectTrajectory(ros::Time timeStamp, const SystemObservation &observation)
    {
      // Marker visualization
      visualization_msgs::Marker marker;
      marker.header.frame_id = frameId_;
      marker.header.stamp = timeStamp;
      marker.ns = "object";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = observation.state(0);
      marker.pose.position.y = observation.state(1);
      marker.pose.position.z = 0.25;

      Eigen::Matrix<scalar_t, 3, 1> euler;
      euler << observation.state(2), 0.0, 0.0;

      const Eigen::Quaternion<scalar_t> quat = getQuaternionFromEulerAnglesZyx(euler); // (yaw, pitch, roll
      marker.pose.orientation.x = quat.x();
      marker.pose.orientation.y = quat.y();
      marker.pose.orientation.z = quat.z();
      marker.pose.orientation.w = quat.w();

      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;

      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      return marker;
    }

    visualization_msgs::Marker ObjectDummyVisualization::ObjectTarget(ros::Time timeStamp, const CommandData &command)
    {
      const auto &targetTrajectories = command.mpcTargetTrajectories_;

      // Marker visualization
      visualization_msgs::Marker marker;
      marker.header.frame_id = frameId_;
      marker.header.stamp = timeStamp;
      marker.ns = "target";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = targetTrajectories.stateTrajectory[1](0);
      marker.pose.position.y = targetTrajectories.stateTrajectory[1](1);
      marker.pose.position.z = 0.25;

      Eigen::Matrix<scalar_t, 3, 1> euler;
      euler << targetTrajectories.stateTrajectory[1](2), 0.0, 0.0;

      const Eigen::Quaternion<scalar_t> quat = getQuaternionFromEulerAnglesZyx(euler); // (yaw, pitch, roll
      marker.pose.orientation.x = quat.x();
      marker.pose.orientation.y = quat.y();
      marker.pose.orientation.z = quat.z();
      marker.pose.orientation.w = quat.w();

      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;

      marker.color.a = 0.2; // Don't forget to set the alpha!
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;

      return marker;
    }

    visualization_msgs::MarkerArray ObjectDummyVisualization::wrench(ros::Time timeStamp, const SystemObservation &observation)
    {

      visualization_msgs::MarkerArray markerArray;

      for (int i = 0; i < AGENT_COUNT; i++)
      {
        // Marker visualization
        visualization_msgs::Marker marker;
        marker.header.frame_id = frameId_;
        marker.header.stamp = timeStamp;
        marker.ns = "wrench";
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        Eigen::Matrix<scalar_t, 3, 1> euler;
        euler << observation.state(2) + params_.agents_init_yaw_[i], 0.0, 0.0;
        Eigen::Matrix3d rotmat = getRotationMatrixFromZyxEulerAngles(euler); // (yaw, pitch, roll)

        auto scaled_input = observation.input(i) / 80;

        Eigen::Matrix<scalar_t, 3, 1> corrected_position = rotmat * (Eigen::Matrix<scalar_t, 3, 1>() << -0.25 - scaled_input,
                                                                     observation.input(AGENT_COUNT + i), 0.0)
                                                                        .finished();

        marker.pose.position.x = observation.state(0) + corrected_position(0);
        marker.pose.position.y = observation.state(1) + corrected_position(1);
        marker.pose.position.z = 0.25;

        const Eigen::Quaternion<scalar_t> quat = getQuaternionFromEulerAnglesZyx(euler); // (yaw, pitch, roll)
        marker.pose.orientation.x = quat.x();
        marker.pose.orientation.y = quat.y();
        marker.pose.orientation.z = quat.z();
        marker.pose.orientation.w = quat.w();

        marker.scale.x = scaled_input;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        markerArray.markers.push_back(marker);
      }

      return markerArray;
    }

    visualization_msgs::MarkerArray ObjectDummyVisualization::obstacles(ros::Time timeStamp)
    {

      visualization_msgs::MarkerArray markerArray;

      // Marker visualization
      visualization_msgs::Marker marker;
      marker.header.frame_id = frameId_;
      marker.header.stamp = timeStamp;
      marker.ns = "obstacles";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::CUBE_LIST;
      marker.action = visualization_msgs::Marker::ADD;

      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;

      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Define a list of points
      for (int i = 0; i < OBSTACLE_COUNT; i++)
      {
        geometry_msgs::Point p;
        p.x = params_.obstacles_(i, 0);
        p.y = params_.obstacles_(i, 1);
        p.z = 0.25;
        marker.points.push_back(p);
      }

      markerArray.markers.push_back(marker);

      //Marker visualization
      visualization_msgs::Marker marker2;
      marker2.header.frame_id = frameId_;
      marker2.header.stamp = timeStamp;
      marker2.ns = "obstacles";
      marker2.id = 1;
      marker2.type = visualization_msgs::Marker::SPHERE_LIST;
      marker2.action = visualization_msgs::Marker::ADD;

      marker2.scale.x = 1;
      marker2.scale.y = 1;
      marker2.scale.z = 1;

      marker2.color.a = 0.7; // Don't forget to set the alpha!
      marker2.color.r = 0.0;
      marker2.color.g = 1.0;
      marker2.color.b = 0.0;

      marker2.pose.orientation.x = 0.0;
      marker2.pose.orientation.y = 0.0;
      marker2.pose.orientation.z = 0.0;
      marker2.pose.orientation.w = 1.0;

      // Define a list of points
      for (int i = 0; i < OBSTACLE_COUNT; i++)
      {
        geometry_msgs::Point p;
        p.x = params_.obstacles_(i, 0);
        p.y = params_.obstacles_(i, 1);
        p.z = 0.25;
        marker2.points.push_back(p);
      }

      // markerArray.markers.push_back(marker2);

      return markerArray;
    }

  } // namespace object_manipulation
} // namespace ocs2
