
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <ocs2_core/Types.h>

namespace ocs2
{
    namespace object_manipulation
    {

        using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
        using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
        using quaternion_t = Eigen::Quaternion<scalar_t>;

        struct StreamedData
        {
            vector3_t position;
            quaternion_t quaternion;
            matrix3_t rotmat;
            vector3_t rpy;
            vector3_t v_world;
            vector3_t v_body;
            vector3_t omega_world;
            vector3_t omega_body;
        };

        class StateEstimation
        {
        public:
            StateEstimation(){
            // Create a subscriber to the gazebo_model_state topic
            modelStateSub = nh.subscribe("/gazebo/model_states", 10, &StateEstimation::modelStateCallback, this);
        };

            void modelStateCallback(const gazebo_msgs::ModelStates &msg){
            int model_index;

            for (int i = 0; i < msg.name.size(); i++)
            {
                if (msg.name[i] == "object")
                {
                    model_index = i;
                }
            }
            object_data.position(0) = msg.pose[model_index].position.x;
            object_data.position(1) = msg.pose[model_index].position.y;
            object_data.position(2) = msg.pose[model_index].position.z;

            object_data.quaternion = quaternion_t(msg.pose[model_index].orientation.w, msg.pose[model_index].orientation.x,
                                                  msg.pose[model_index].orientation.y, msg.pose[model_index].orientation.z);

            object_data.rotmat = object_data.quaternion.toRotationMatrix();
            object_data.rpy = object_data.rotmat.eulerAngles(0, 1, 2);

            object_data.v_world(0) = msg.twist[model_index].linear.x;
            object_data.v_world(1) = msg.twist[model_index].linear.y;
            object_data.v_world(2) = msg.twist[model_index].linear.z;

            object_data.v_body = object_data.rotmat * object_data.v_world;

            object_data.omega_world(0) = msg.twist[model_index].angular.x;
            object_data.omega_world(1) = msg.twist[model_index].angular.y;
            object_data.omega_world(2) = msg.twist[model_index].angular.z;

            object_data.omega_body = object_data.rotmat * object_data.omega_world;
            ROS_INFO("I heard: x =%f, y=%f, z=%f", msg.pose[model_index].position.x, msg.pose[model_index].position.y, msg.pose[model_index].position.z);
        };
            
        private:
            ros::NodeHandle nh;
            ros::Subscriber modelStateSub;
            StreamedData object_data;
        };

    } // namespace object_manipulation
} // namespace ocs2

