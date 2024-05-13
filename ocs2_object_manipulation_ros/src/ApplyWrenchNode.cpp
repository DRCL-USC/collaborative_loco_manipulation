#include <ros/ros.h>
#include <gazebo_msgs/ApplyBodyWrench.h> // Replace with the appropriate service message header
#include <ocs2_core/Types.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

using namespace ocs2;

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "my_node");

    // Initialize the ROS node handle
    ros::NodeHandle nh;
    
    // Create a wrench vector
    vector_t wrench_w;
    vector_t wrench_b;

    // Create a publish message
    geometry_msgs::Wrench wrench_msg;
    ros::Publisher pub = nh.advertise<geometry_msgs::Wrench>("/robot_1/wrench", 1);
    Eigen::Matrix<scalar_t, 2, 2> rotmat_2d;
    scalar_t yaw;

    auto WrenchCallback = [&](const ocs2_msgs::mpc_observation::ConstPtr &msg)
    {
        std::unique_ptr<ocs2::SystemObservation> observationPtr_(new ocs2::SystemObservation(ocs2::ros_msg_conversions::readObservationMsg(*msg)));
        
        wrench_w = observationPtr_->input;

        yaw = observationPtr_->state(2);
        rotmat_2d << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
        wrench_b = rotmat_2d.transpose() * wrench_w.head(2);

        wrench_msg.force.x = wrench_b(0);
        wrench_msg.force.y = wrench_b(1);
        wrench_msg.force.z = 0;
        wrench_msg.torque.x = 0;
        wrench_msg.torque.y = 0;
        wrench_msg.torque.z = wrench_w(2);

        pub.publish(wrench_msg);
        
        ros::Duration(0.01).sleep();
    };

    ros::Subscriber sub = nh.subscribe<ocs2_msgs::mpc_observation>("/object_mpc_observation", 1, WrenchCallback);

    // Spin
    ros::spin();
    
    return 0;
}
