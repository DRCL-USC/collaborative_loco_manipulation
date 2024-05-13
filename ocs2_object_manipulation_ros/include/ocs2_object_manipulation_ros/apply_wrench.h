
#ifndef GAZEBO_ROS_FORCE_HH
#define GAZEBO_ROS_FORCE_HH

#include <string>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Pose.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

namespace ocs2_object_manipulation_ros
{
    using namespace gazebo;

    class ApplyBodyForce : public ModelPlugin
    {
        /// \brief Constructor
    public:
        ApplyBodyForce();

        /// \brief Destructor
    public:
        virtual ~ApplyBodyForce();

        // Documentation inherited
    protected:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        // Documentation inherited
    protected:
        virtual void UpdateChild();

        /// \brief call back when a Wrench message is published
        /// \param[in] _msg The Incoming ROS message representing the new force to exert.
    private:
        void UpdateObjectForce(const geometry_msgs::Wrench::ConstPtr &_msg);
        void UpdateContactPoint(const geometry_msgs::Pose::ConstPtr &_msg);

        /// \brief The custom callback queue thread function.
    private:
        void QueueThread();

        /// \brief A pointer to the gazebo world.
    private:
        physics::WorldPtr world_;

        /// \brief A pointer to the Link, where force is applied
    private:
        physics::LinkPtr link_;

        /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
    private:
        ros::NodeHandle *rosnode_;

    private:
        ros::Subscriber sub_;

        /// \brief A mutex to lock access to fields that are used in ROS message callbacks
    private:
        boost::mutex lock_;

        /// \brief ROS Wrench topic name inputs
    private:
        std::string force_topic_name_;
        std::string reference_point_topic_name_;
        /// \brief The Link this plugin is attached to, and will exert forces on.
    private:
        std::string link_name_;

        /// \brief for setting ROS name space
    private:
        std::string robot_namespace_;

        // Custom Callback Queue
    private:
        ros::CallbackQueue queue_;
        ros::CallbackQueue queue2_;
        /// \brief Thead object for the running callback Thread.
    private:
        boost::thread callback_queue_thread_;
        /// \brief Container for the wrench force that this plugin exerts on the body.
    private:
        geometry_msgs::Wrench wrench_msg_;
        geometry_msgs::Pose pose_msg_;

        // Pointer to the update event connection
    private:
        event::ConnectionPtr update_connection_;
    };
}
#endif
