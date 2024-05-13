
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
            vector_t state;
        };

        template <typename T>
        T square(T a)
        {
            return a * a;
        }

        template <typename SCALAR_T>
        Eigen::Matrix<SCALAR_T, 3, 1> quatToXyz(const Eigen::Quaternion<SCALAR_T> &q)
        {
            Eigen::Matrix<SCALAR_T, 3, 1> xyz;

            SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
            xyz(2) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
            xyz(1) = std::asin(as);
            xyz(0) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
            return xyz;
        }

        class StateEstimation
        {
        public:
            StateEstimation()
            {
                // Create a subscriber to the gazebo_model_state topic
                modelStateSub = nh.subscribe("/gazebo/model_states", 10, &StateEstimation::modelStateCallback, this);
                object_data.state = vector_t::Zero(6);
            };

            void modelStateCallback(const gazebo_msgs::ModelStates &msg)
            {
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
                // object_data.rpy = object_data.rotmat.eulerAngles(0, 1, 2);
                object_data.rpy = quatToXyz(object_data.quaternion);

                object_data.v_world(0) = msg.twist[model_index].linear.x;
                object_data.v_world(1) = msg.twist[model_index].linear.y;
                object_data.v_world(2) = msg.twist[model_index].linear.z;

                object_data.v_body = object_data.rotmat * object_data.v_world;

                object_data.omega_world(0) = msg.twist[model_index].angular.x;
                object_data.omega_world(1) = msg.twist[model_index].angular.y;
                object_data.omega_world(2) = msg.twist[model_index].angular.z;

                object_data.omega_body = object_data.rotmat * object_data.omega_world;

                object_data.state << object_data.position(0), object_data.position(1), object_data.rpy(2),
                    object_data.v_world(0), object_data.v_world(1), object_data.omega_world(2);

                // ROS_INFO("I heard: x =%f, y=%f, yaw=%f , vx=%f, vy=%f, omega=%f",
                //          object_data.state[0], object_data.state[1], object_data.state[2], object_data.state[3], object_data.state[4], object_data.state[5]);
            };

            StreamedData object_data;

        private:
            ros::NodeHandle nh;
            ros::Subscriber modelStateSub;
        };

    } // namespace object_manipulation
} // namespace ocs2
