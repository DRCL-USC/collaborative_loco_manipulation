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

    // Create a client to call the service
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

    // Create a service request object
    gazebo_msgs::ApplyBodyWrench srv[3];
    
    // Create a wrench vector
    ocs2::vector_t wrench;

    auto WrenchCallback = [&](const ocs2_msgs::mpc_observation::ConstPtr &msg)
    {
        std::unique_ptr<ocs2::SystemObservation> observationPtr_(new ocs2::SystemObservation(ocs2::ros_msg_conversions::readObservationMsg(*msg)));
        
        wrench = observationPtr_->input;

        srv[0].request.body_name = "object::object";
        srv[0].request.reference_frame = "object::object";
        // srv.request.reference_point = value3;
        srv[0].request.wrench.force.x = wrench(0);
        srv[0].request.start_time = ros::Time(0.0);
        srv[0].request.duration = ros::Duration(-1.0);

        srv[1].request.body_name = "object::dummy";
        srv[1].request.reference_frame = "object::dummy";
        // srv.request.reference_point = value3;
        srv[1].request.wrench.force.y = wrench(1);
        srv[1].request.start_time = ros::Time(0.0);
        srv[1].request.duration = ros::Duration(-1.0);

        srv[2].request.body_name = "object::dummy_2";
        srv[2].request.reference_frame = "object::dummy_2";
        // srv.request.reference_point = value3;
        srv[2].request.wrench.torque.z = wrench(2);
        srv[2].request.start_time = ros::Time(0.0);
        srv[2].request.duration = ros::Duration(-1.0);

        for (int i = 0; i < 3; i++)
        {
            // Call the service
            if (!client.call(srv[i]))
            {
                ROS_ERROR("Failed to call service");
            }
        }
        
        ROS_INFO("Wrench: %f, %f, %f", wrench(0), wrench(1), wrench(2));
        
        ros::Duration(0.01).sleep();
    };

    ros::Subscriber sub = nh.subscribe<ocs2_msgs::mpc_observation>("/object_mpc_observation", 1, WrenchCallback);

    // Spin
    ros::spin();
    
    return 0;
}
