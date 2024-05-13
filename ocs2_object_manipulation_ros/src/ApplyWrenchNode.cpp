#include <ros/ros.h>
#include <gazebo_msgs/ApplyBodyWrench.h> // Replace with the appropriate service message header
#include <ocs2_core/Types.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>


int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "my_node");

    // Initialize the ROS node handle
    ros::NodeHandle nh;
    
    // Create a wrench vector
    ocs2::vector_t wrench;

    // Create a publish message
    geometry_msgs::Wrench wrench_msg;
    ros::Publisher pub = nh.advertise<geometry_msgs::Wrench>("/robot_1/wrench", 1);

    auto WrenchCallback = [&](const ocs2_msgs::mpc_observation::ConstPtr &msg)
    {
        std::unique_ptr<ocs2::SystemObservation> observationPtr_(new ocs2::SystemObservation(ocs2::ros_msg_conversions::readObservationMsg(*msg)));
        
        wrench = observationPtr_->input;
        wrench_msg.force.x = wrench(0);
        wrench_msg.force.y = wrench(1);
        wrench_msg.force.z = 0;
        wrench_msg.torque.x = 0;
        wrench_msg.torque.y = 0;
        wrench_msg.torque.z = wrench(2);

        pub.publish(wrench_msg);
        
        ros::Duration(0.01).sleep();
    };

    ros::Subscriber sub = nh.subscribe<ocs2_msgs::mpc_observation>("/object_mpc_observation", 1, WrenchCallback);

    // Spin
    ros::spin();
    
    return 0;
}
