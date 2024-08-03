
#include <ros/ros.h>
#include <ocs2_core/Types.h>
#include <visualization_msgs/MarkerArray.h>
#include <ocs2_object_manipulation/ObjectInterface.h>
#include <std_msgs/Bool.h>

class ObstacleVisualization
{
public:
    std::string frameId_ = "world"; // Frame name all messages are published in                                                        // LineThickness for trajectories

    ObstacleVisualization(ros::NodeHandle &nodehandle, std::shared_ptr<ocs2::object_manipulation::Obstacles> obstaclesPtr)
        : obstaclesPtr_(std::move(obstaclesPtr)), nh(nodehandle)
    {
        obstaclesPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_markers", 0);
        obstaclesSubscriber_ = nh.subscribe("/obstacle_visualizer", 1, &ObstacleVisualization::callback, this);
    }

    void callback(const std_msgs::Bool::ConstPtr &msg)
    {

        auto obstacles_pose = obstaclesPtr_->getObstacles();
        visualization_msgs::MarkerArray markerArray;
        ros::Time timeStamp = ros::Time::now();

        // Marker visualization
        visualization_msgs::Marker marker;
        marker.header.frame_id = frameId_;
        marker.header.stamp = timeStamp;
        marker.ns = "obstacles";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.25;
        marker.scale.y = 0.35;
        marker.scale.z = 0.55;

        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Define a list of points
        for (int i = 0; i < obstacles_pose.size(); i++)
        {
            geometry_msgs::Point p;
            p.x = obstacles_pose[i].first;
            p.y = obstacles_pose[i].second;
            p.z = 0.275; // magic number
            marker.points.push_back(p);
        }

        markerArray.markers.push_back(marker);

        obstaclesPublisher_.publish(markerArray);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber obstaclesSubscriber_;
    ros::Publisher obstaclesPublisher_;
    std::shared_ptr<ocs2::object_manipulation::Obstacles> obstaclesPtr_;
};
