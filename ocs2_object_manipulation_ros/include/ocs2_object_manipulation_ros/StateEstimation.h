
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <ocs2_core/Types.h>
#include <geometry_msgs/PoseStamped.h>

namespace ocs2
{
    namespace object_manipulation
    {

        using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
        using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
        using quaternion_t = Eigen::Quaternion<scalar_t>;

        struct Data
        {
            double time;
            vector3_t position;
            quaternion_t quaternion;
            matrix3_t rotmat;
            vector3_t rpy;
            vector3_t v_world;
            vector3_t v_body;
            vector3_t omega_world;
            vector3_t omega_body;
            vector_t state;
            Data()
            {
                time = 0.0;
                position = vector3_t::Zero();
                quaternion = quaternion_t::Identity();
                rotmat = matrix3_t::Identity();
                rpy = vector3_t::Zero();
                v_world = vector3_t::Zero();
                v_body = vector3_t::Zero();
                omega_world = vector3_t::Zero();
                omega_body = vector3_t::Zero();
                state = vector_t::Zero(STATE_DIM);
            }
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

        class StateEstimationBase
        {
        public:
            StateEstimationBase() {};
            bool isInitialized()
            {
                return _data.time != 0.0;
            }
            Data _data;

        protected:
            ros::NodeHandle nh;
            ros::Subscriber Sub;
        };

        class StateEstimationCheater : public StateEstimationBase
        {
        public:
            StateEstimationCheater()
            {
                // Create a subscriber to the gazebo_model_state topic
                Sub = nh.subscribe("/gazebo/model_states", 10, &StateEstimationCheater::modelStateCallback, this);
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
                _data.position(0) = msg.pose[model_index].position.x;
                _data.position(1) = msg.pose[model_index].position.y;
                _data.position(2) = msg.pose[model_index].position.z;

                _data.quaternion = quaternion_t(msg.pose[model_index].orientation.w, msg.pose[model_index].orientation.x,
                                                msg.pose[model_index].orientation.y, msg.pose[model_index].orientation.z);

                _data.rotmat = _data.quaternion.toRotationMatrix();
                // _data.rpy = _data.rotmat.eulerAngles(0, 1, 2);
                _data.rpy = quatToXyz(_data.quaternion);

                _data.v_world(0) = msg.twist[model_index].linear.x;
                _data.v_world(1) = msg.twist[model_index].linear.y;
                _data.v_world(2) = msg.twist[model_index].linear.z;

                _data.v_body = _data.rotmat * _data.v_world;

                _data.omega_world(0) = msg.twist[model_index].angular.x;
                _data.omega_world(1) = msg.twist[model_index].angular.y;
                _data.omega_world(2) = msg.twist[model_index].angular.z;

                _data.omega_body = _data.rotmat * _data.omega_world;

                _data.state << _data.position(0), _data.position(1), _data.rpy(2),
                    _data.v_world(0), _data.v_world(1), _data.omega_world(2);

                // ROS_INFO("I heard: x =%f, y=%f, yaw=%f , vx=%f, vy=%f, omega=%f",
                //          _data.state[0], _data.state[1], _data.state[2], _data.state[3], _data.state[4], _data.state[5]);
            };
        };

        class StateEstimationMocap : public StateEstimationBase
        {
        public:
            StateEstimationMocap(std::string object_name)
            {
                // Create a subscriber to mocap topic
                Sub = nh.subscribe("/vrpn_client_node/" + object_name + "/pose", 1, &StateEstimationMocap::mocapCallback, this);
            };

            void mocapCallback(const geometry_msgs::PoseStamped &msg)
            {
                _data.time = msg.header.stamp.toSec();

                _data.position[0] = msg.pose.position.x;
                _data.position[1] = msg.pose.position.y;
                _data.position[2] = msg.pose.position.z;

                // ROS_INFO("I heard: x =%f, y=%f, z=%f", _data.position[0], _data.position[1], _data.position[2]);

                _data.quaternion = quaternion_t(msg.pose.orientation.w, msg.pose.orientation.x,
                                                msg.pose.orientation.y, msg.pose.orientation.z);

                _data.rotmat = _data.quaternion.toRotationMatrix();
                _data.rpy = quatToXyz(_data.quaternion);

                LinearVelocityCalc();
                AngularVelocityCalc();

                _data.state << _data.position(0), _data.position(1), _data.rpy(2),
                    _data.v_world(0), _data.v_world(1), _data.omega_world(2);

                _dataPrev.time = _data.time;
            }

            void LinearVelocityCalc()
            {
                double dt = _data.time - _dataPrev.time;

                for (int i = 0; i < 3; i++)
                {
                    _data.v_world[i] = (_data.position[i] - _dataPrev.position[i]) / dt;
                }
                _data.v_body = _data.rotmat * _data.v_world;

                for (int i = 0; i < 3; i++)
                {
                    _dataPrev.position[i] = _data.position[i];
                }
            }

            void AngularVelocityCalc()
            {
                double dt = _data.time - _dataPrev.time;

                _data.omega_world[0] = (2 / dt) * (_dataPrev.quaternion.w() * _data.quaternion.x() - _dataPrev.quaternion.x() * _data.quaternion.w() - _dataPrev.quaternion.y() * _data.quaternion.z() + _dataPrev.quaternion.z() * _data.quaternion.y());
                _data.omega_world[1] = (2 / dt) * (_dataPrev.quaternion.w() * _data.quaternion.y() + _dataPrev.quaternion.x() * _data.quaternion.z() - _dataPrev.quaternion.y() * _data.quaternion.w() - _dataPrev.quaternion.z() * _data.quaternion.x());
                _data.omega_world[2] = (2 / dt) * (_dataPrev.quaternion.w() * _data.quaternion.z() - _dataPrev.quaternion.x() * _data.quaternion.y() + _dataPrev.quaternion.y() * _data.quaternion.x() - _dataPrev.quaternion.z() * _data.quaternion.w());

                _data.omega_body = _data.rotmat * _data.omega_world;
                _dataPrev.quaternion = _data.quaternion;
            }

        private:
            Data _dataPrev;
        };

    } // namespace object_manipulation
} // namespace ocs2
