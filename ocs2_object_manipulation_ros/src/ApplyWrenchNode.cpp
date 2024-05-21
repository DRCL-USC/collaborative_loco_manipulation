#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <ocs2_core/Types.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <ocs2_object_manipulation/definitions.h>

using namespace ocs2;
using namespace object_manipulation;

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "my_node");

    // Initialize the ROS node handle
    ros::NodeHandle nh;

    // Create a publish message
    geometry_msgs::Wrench wrench_msg[2];
    geometry_msgs::Pose pose_msg[2];
    ros::Publisher pub_wrench[2], pub_pose[2];
    pub_wrench[0] = nh.advertise<geometry_msgs::Wrench>("/robot_1/wrench", 1);
    pub_wrench[1] = nh.advertise<geometry_msgs::Wrench>("/robot_2/wrench", 1);
    pub_pose[0] = nh.advertise<geometry_msgs::Pose>("/robot_1/contactPoint", 1);
    pub_pose[1] = nh.advertise<geometry_msgs::Pose>("/robot_2/contactPoint", 1);

    auto WrenchCallback = [&](const ocs2_msgs::mpc_observation::ConstPtr &msg)
    {
        std::unique_ptr<ocs2::SystemObservation> observationPtr_(new ocs2::SystemObservation(ocs2::ros_msg_conversions::readObservationMsg(*msg)));

        for (int i = 0; i < AGENT_COUNT; i++)
        {
            wrench_msg[i].force.x = std::fmax(observationPtr_->input(i), 0.0);
            wrench_msg[i].force.y = 0;
            wrench_msg[i].force.z = 0;
            wrench_msg[i].torque.x = 0;
            wrench_msg[i].torque.y = 0;
            wrench_msg[i].torque.z = 0;
            pub_wrench[i].publish(wrench_msg[i]);

            pose_msg[i].position.x = 0.0;
            pose_msg[i].position.y = observationPtr_->input(2+i);
            pose_msg[i].position.z = 0.0;
            pub_pose[i].publish(pose_msg[i]);
        }

        ros::Duration(0.01).sleep();
    };

    ros::Subscriber sub = nh.subscribe<ocs2_msgs::mpc_observation>("/object_mpc_observation", 1, WrenchCallback);

    // Spin
    ros::spin();

    return 0;
}
