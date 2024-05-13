
#include <algorithm>
#include <assert.h>
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

namespace gazebo
{

    class ApplyBodyForce : public ModelPlugin
    {
    public:
        // Constructor
        ApplyBodyForce()
        {
            this->wrench_msg_.force.x = 0;
            this->wrench_msg_.force.y = 0;
            this->wrench_msg_.force.z = 0;
            this->wrench_msg_.torque.x = 0;
            this->wrench_msg_.torque.y = 0;
            this->wrench_msg_.torque.z = 0;
            this->pose_msg_.position.x = 0;
            this->pose_msg_.position.y = 0;
            this->pose_msg_.position.z = 0;
        }

        // Destructor
        ~ApplyBodyForce()
        {
            this->update_connection_.reset();

            // Custom Callback Queue
            this->queue_[0].clear();
            this->queue_[0].disable();
            this->queue_[1].clear();
            this->queue_[1].disable();
            this->rosnode_->shutdown();
            this->callback_queue_thread_.join();

            delete this->rosnode_;
        }

    protected:
        // Load the controller
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Get the world name.
            this->world_ = _model->GetWorld();

            // load parameters
            this->robot_namespace_ = "";
            if (_sdf->HasElement("robotNamespace"))
                this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

            if (!_sdf->HasElement("bodyName"))
            {
                ROS_FATAL_NAMED("force", "force plugin missing <bodyName>, cannot proceed");
                return;
            }
            else
                this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

            this->link_ = _model->GetLink(this->link_name_);
            if (!this->link_)
            {
                ROS_FATAL_NAMED("force", "gazebo_ros_force plugin error: link named: %s does not exist\n", this->link_name_.c_str());
                return;
            }

            if (!_sdf->HasElement("ForceTopicName"))
            {
                ROS_FATAL_NAMED("force", "force plugin missing <ForceTopicName>, cannot proceed");
                return;
            }
            else
                this->force_topic_name_ = _sdf->GetElement("ForceTopicName")->Get<std::string>();

            if (!_sdf->HasElement("referencePointTopicName"))
            {
                ROS_FATAL_NAMED("force", "force plugin missing <referencePointTopicName>, cannot proceed");
                return;
            }
            else
                this->reference_point_topic_name_ = _sdf->GetElement("referencePointTopicName")->Get<std::string>();

            // Make sure the ROS node for Gazebo has already been initialized
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM_NAMED("force", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                                    << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }

            this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

            // Custom Callback Queue
            ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Wrench>(
                this->force_topic_name_, 1,
                boost::bind(&ApplyBodyForce::UpdateObjectForce, this, _1),
                ros::VoidPtr(), &this->queue_[0]);

            ros::SubscribeOptions fo = ros::SubscribeOptions::create<geometry_msgs::Pose>(
                this->reference_point_topic_name_, 1,
                boost::bind(&ApplyBodyForce::UpdateContactPoint, this, _1),
                ros::VoidPtr(), &this->queue_[1]);

            this->sub_[0] = this->rosnode_->subscribe(so);
            this->sub_[1] = this->rosnode_->subscribe(fo);

            // Custom Callback Queue
            this->callback_queue_thread_ = boost::thread(boost::bind(&ApplyBodyForce::QueueThread, this));

            // New Mechanism for Updating every World Cycle
            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&ApplyBodyForce::UpdateChild, this));
        }

        // Update the controller
        void UpdateObjectForce(const geometry_msgs::Wrench::ConstPtr &_msg)
        {
            this->wrench_msg_.force.x = _msg->force.x;
            this->wrench_msg_.force.y = _msg->force.y;
            this->wrench_msg_.force.z = _msg->force.z;
            this->wrench_msg_.torque.x = _msg->torque.x;
            this->wrench_msg_.torque.y = _msg->torque.y;
            this->wrench_msg_.torque.z = _msg->torque.z;
        }

        void UpdateContactPoint(const geometry_msgs::Pose::ConstPtr &_msg)
        {
            this->pose_msg_.position.x = _msg->position.x;
            this->pose_msg_.position.y = _msg->position.y;
            this->pose_msg_.position.z = _msg->position.z;
        }

        // Update the controller
        void UpdateChild()
        {
            this->lock_.lock();
            ignition::math::Vector3d force(this->wrench_msg_.force.x, this->wrench_msg_.force.y, this->wrench_msg_.force.z);
            ignition::math::Vector3d torque(this->wrench_msg_.torque.x, this->wrench_msg_.torque.y, this->wrench_msg_.torque.z);
            ignition::math::Vector3d pose(this->pose_msg_.position.x, this->pose_msg_.position.y, this->pose_msg_.position.z);
            this->link_->AddLinkForce(force, pose);
            this->link_->AddRelativeTorque(torque);

            this->lock_.unlock();
        }

        // Custom Callback Queue
        // custom callback queue thread
        void QueueThread()
        {
            static const double timeout = 0.01;

            while (this->rosnode_->ok())
            {
                this->queue_[0].callAvailable(ros::WallDuration(timeout));
                this->queue_[1].callAvailable(ros::WallDuration(timeout));
            }
        }

    private:
        physics::WorldPtr world_;
        physics::LinkPtr link_;
        ros::NodeHandle *rosnode_;
        ros::Subscriber sub_[2];
        boost::mutex lock_;
        std::string force_topic_name_;
        std::string reference_point_topic_name_;
        std::string link_name_;
        std::string robot_namespace_;
        ros::CallbackQueue queue_[2];
        boost::thread callback_queue_thread_;
        geometry_msgs::Wrench wrench_msg_;
        geometry_msgs::Pose pose_msg_;
        event::ConnectionPtr update_connection_;
    };
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ApplyBodyForce);

}