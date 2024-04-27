#include <ros/ros.h>
#include <ocs2_object_manipulation_ros/StateEstimation.h>

using namespace ocs2::object_manipulation;

int main(int argc, char** argv)
{
    
    // Initialize ROS node
    ros::init(argc, argv, "state_estimation_node");
    ros::NodeHandle nh;

    // Create an instance of the StateEstimation class
    StateEstimation stateEstimation;

    // TODO: Add your code for state estimation here

    // Spin the ROS node
    ros::spin();

    return 0;
}