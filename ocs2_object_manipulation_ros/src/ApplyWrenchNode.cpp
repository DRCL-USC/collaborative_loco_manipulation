#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <ocs2_core/Types.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <ocs2_object_manipulation/definitions.h>
#include <ros/package.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_object_manipulation/LowPassFilter.h>

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
    geometry_msgs::Twist twist_msg[2];
    ros::Publisher pub_wrench[2], pub_pose[2], pub_twist[2];
    pub_wrench[0] = nh.advertise<geometry_msgs::Wrench>("/robot_1/wrench", 1);
    pub_wrench[1] = nh.advertise<geometry_msgs::Wrench>("/robot_2/wrench", 1);
    pub_pose[0] = nh.advertise<geometry_msgs::Pose>("/robot_1/contactPoint", 1);
    pub_pose[1] = nh.advertise<geometry_msgs::Pose>("/robot_2/contactPoint", 1);
    pub_twist[0] = nh.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 1);
    pub_twist[1] = nh.advertise<geometry_msgs::Twist>("/robot_2/cmd_vel", 1);

    bool start = false;
    TargetTrajectories targetTrajectories;
    targetTrajectories.stateTrajectory.push_back(vector_t().setZero(6));
    scalar_array_t agents_init_yaw_ = {0.0, 0.0};
    const std::string taskFile = ros::package::getPath("ocs2_object_manipulation") + "/config/mpc/task.info";
    loadData::loadStdVector(taskFile, "yaw_init", agents_init_yaw_, false);
    LowPassFilter<scalar_t> lpf(0.1);

    auto WrenchCallback = [&](const ocs2_msgs::mpc_observation::ConstPtr &msg)
    {
        std::unique_ptr<ocs2::SystemObservation> observationPtr_(new ocs2::SystemObservation(ocs2::ros_msg_conversions::readObservationMsg(*msg)));

        scalar_t distance = sqrt(pow(observationPtr_->state(0) - targetTrajectories.stateTrajectory.back()[0], 2) +
                                 pow(observationPtr_->state(1) - targetTrajectories.stateTrajectory.back()[1], 2));

        if (distance < 0.4) // magic number
        {
            start = false;
        }

        if (start)
        {
            for (int i = 0; i < AGENT_COUNT; i++)
            {

                Eigen::Matrix<scalar_t, 3, 1> euler;
                euler << observationPtr_->state(2) + agents_init_yaw_[i], 0.0, 0.0;
                Eigen::Matrix3d rotmat = getRotationMatrixFromZyxEulerAngles(euler); // (yaw, pitch, roll)

                Eigen::Matrix<scalar_t, 3, 1> obj_pose_robot_frame = rotmat.transpose() * (Eigen::Matrix<scalar_t, 3, 1>() << observationPtr_->state(0),
                                                                                           observationPtr_->state(1), 0.0)
                                                                                              .finished();

                Eigen::Matrix<scalar_t, 3, 1> obj_vel_robot_frame = rotmat.transpose() * (Eigen::Matrix<scalar_t, 3, 1>() << observationPtr_->state(3),
                                                                                          observationPtr_->state(4), 0.0)
                                                                                             .finished();

                wrench_msg[i].force.x = std::fmax(observationPtr_->input(i), 0.0);
                wrench_msg[i].force.y = 0;
                wrench_msg[i].force.z = 0;
                wrench_msg[i].torque.x = 0;
                wrench_msg[i].torque.y = 0;
                wrench_msg[i].torque.z = 0;
                pub_wrench[i].publish(wrench_msg[i]);

                pose_msg[i].position.x = obj_pose_robot_frame(0) - 0.25; // 0.25 is the radius length - magic number
                pose_msg[i].position.y = obj_pose_robot_frame(1) + lpf.process(observationPtr_->input(2 + i));
                pose_msg[i].position.z = 0.0;
                pub_pose[i].publish(pose_msg[i]);

                twist_msg[i].linear.x = obj_vel_robot_frame(0);
                twist_msg[i].linear.y = obj_vel_robot_frame(1);
                twist_msg[i].linear.z = 0;
                twist_msg[i].angular.x = 0;
                twist_msg[i].angular.y = 0;
                twist_msg[i].angular.z = observationPtr_->state(5);
                pub_twist[i].publish(twist_msg[i]);
            }

            ros::Duration(0.01).sleep();
        }
    };

    ros::Subscriber sub = nh.subscribe<ocs2_msgs::mpc_observation>("/object_mpc_observation", 1, WrenchCallback);

    auto targetCallback = [&](const ocs2_msgs::mpc_target_trajectories::ConstPtr &msg)
    {
        start = true;
        targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(*msg);
    };

    ros::Subscriber start_sub = nh.subscribe<ocs2_msgs::mpc_target_trajectories>("object_mpc_target", 1, targetCallback);

    // Spin
    ros::spin();

    return 0;
}
