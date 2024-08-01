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
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <string>
#include <pthread.h>
#include <stdint.h>
#include <fstream>
#include <netinet/in.h>
#include <arpa/inet.h>

using namespace ocs2;
using namespace object_manipulation;

struct DataSend
{
    float quaternion[4];
    float position[3];
    float velocity[3];
    float force[3];
    float omega[3];
    DataSend()
    {
        for (int i = 0; i < 4; i++)
        {
            quaternion[i] = 0;
        }
        for (int i = 0; i < 3; i++)
        {
            position[i] = 0;
            velocity[i] = 0;
            force[i] = 0;
            omega[i] = 0;
        }
    }
};

std::map<char, std::string> modeBindings = {
    {'1', "START"},
    {'2', "STOP"}};

char getKey(double timeout)
{
    char key = 0;
    struct termios old_settings, new_settings;
    tcgetattr(STDIN_FILENO, &old_settings);
    new_settings = old_settings;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

    fd_set readfds;
    struct timeval tv;
    tv.tv_sec = static_cast<int>(timeout);
    tv.tv_usec = static_cast<int>((timeout - tv.tv_sec) * 1e6);

    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    int ret = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv);

    if (ret == 1 && FD_ISSET(STDIN_FILENO, &readfds))
    {
        key = getchar();
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
    return key;
}

int setupsocket(const char *serverIP, int port)
{
    // Create a socket
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);

    // Configure the server address
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(port); // Use the same port as the server

    if (inet_pton(AF_INET, serverIP, &(serverAddress.sin_addr)) != 1)
    {
        std::cerr << "Error converting IP address" << std::endl;
        return -1;
    }

    // Connect to the server
    int connectionStatus = connect(clientSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress));

    if (connectionStatus == -1)
    {
        std::cerr << "Connection error." << std::endl;
        return -1;
    }

    std::cout << "Connected to the robot." << std::endl;

    return clientSocket;
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "apply_wrench_node");

    // Initialize the ROS node handle
    ros::NodeHandle nh;

    // communication setup
    std::string mode;
    DataSend dataSend[AGENT_COUNT];
    int robot_socket[AGENT_COUNT];
    robot_socket[0] = setupsocket("127.0.0.1", 8188);
    robot_socket[1] = setupsocket("127.0.0.1", 8198);

    bool start = false;
    TargetTrajectories targetTrajectories;
    targetTrajectories.stateTrajectory.push_back(vector_t().setZero(6));
    scalar_array_t agents_init_yaw_ = {0.0, 0.0};
    scalar_t targetError;
    const std::string taskFile = ros::package::getPath("ocs2_object_manipulation") + "/config/mpc/task.info";
    loadData::loadStdVector(taskFile, "yaw_init", agents_init_yaw_, false);
    loadData::loadCppDataType(taskFile, "targetDisplacementError", targetError);
    LowPassFilter<scalar_t> lpf(0.1);

    auto WrenchCallback = [&](const ocs2_msgs::mpc_observation::ConstPtr &msg)
    {
        std::unique_ptr<ocs2::SystemObservation> observationPtr_(new ocs2::SystemObservation(ocs2::ros_msg_conversions::readObservationMsg(*msg)));

        scalar_t distance = sqrt(pow(observationPtr_->state(0) - targetTrajectories.stateTrajectory.back()[0], 2) +
                                 pow(observationPtr_->state(1) - targetTrajectories.stateTrajectory.back()[1], 2));

        char key = getKey(0.01);
        if (modeBindings.find(key) != modeBindings.end())
        {
            mode = modeBindings[key];
            ROS_INFO("%s", mode.c_str());
        }

        if (distance < targetError || mode == "STOP")
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

                Eigen::Matrix<scalar_t, 3, 1> obj_pose_world_frame = rotmat * (Eigen::Matrix<scalar_t, 3, 1>() << -0.25,
                                                                               lpf.process(observationPtr_->input(2 + i)), 0.0)
                                                                                  .finished(); // 0.25 is the radius length - magic number

                Eigen::Matrix<scalar_t, 3, 1> obj_vel_robot_frame = rotmat.transpose() * (Eigen::Matrix<scalar_t, 3, 1>() << observationPtr_->state(3),
                                                                                          observationPtr_->state(4), 0.0)
                                                                                             .finished();

                dataSend[i].force[0] = 10 + i;
                send(robot_socket[i], &dataSend[i], sizeof(dataSend[i]), 0);
                // wrench_msg[i].force.x = std::fmax(observationPtr_->input(i), 0.0);
                // wrench_msg[i].force.y = 0;
                // wrench_msg[i].force.z = 0;
                // wrench_msg[i].torque.x = 0;
                // wrench_msg[i].torque.y = 0;
                // wrench_msg[i].torque.z = 0;
                // pub_wrench[i].publish(wrench_msg[i]);

                // pose_msg[i].position.x = observationPtr_->state(0) + obj_pose_world_frame(0);
                // pose_msg[i].position.y = observationPtr_->state(1) + obj_pose_world_frame(1);
                // pose_msg[i].position.z = 0.0;
                // pub_pose[i].publish(pose_msg[i]);

                // twist_msg[i].linear.x = obj_vel_robot_frame(0);
                // twist_msg[i].linear.y = obj_vel_robot_frame(1);
                // twist_msg[i].linear.z = 0;
                // twist_msg[i].angular.x = 0;
                // twist_msg[i].angular.y = 0;
                // twist_msg[i].angular.z = observationPtr_->state(5);
                // pub_twist[i].publish(twist_msg[i]);
            }
        }
        else
        {
            for (int i = 0; i < AGENT_COUNT; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    dataSend[i].force[j] = 0;
                    dataSend[i].position[j] = 0;
                    dataSend[i].velocity[j] = 0;
                    dataSend[i].omega[j] = 0;
                }
                send(robot_socket[i], &dataSend[i], sizeof(dataSend[i]), 0);
            }

            ROS_INFO_THROTTLE(1, "Target reached or STOP command received. Waiting for new command.");
        }
        ros::Duration(0.01).sleep();
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
