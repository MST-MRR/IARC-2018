#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
/// \brief A plugin to control a Velodyne sensor.
class VelodynePlugin : public ModelPlugin
{
    /// \brief Constructor
  public:
    VelodynePlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
  public:
    void SetVelocity(const double &_vel)
    {
        this->model->GetJointController()->SetVelocityTarget(
            this->joint->GetScopedName(), _vel);
    }

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Just output a message for now
        std::cerr << "\nThe velodyne plugin is attach to model[" << _model->GetName() << "]\n";
        // Safety check
        if (_model->GetJointCount() == 0)
        {
            std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
            return;
        }

        // Store the model pointer for convenience.
        this->model = _model;

        // Get the first joint. We are making an assumption about the model
        // having one joint that is the rotational joint.
        this->joint = _model->GetJoints()[0];

        // Setup a P-controller, with a gain of 0.1.
        this->pid = common::PID(0.1, 0, 0);

        // Apply the P-controller to the joint.
        this->model->GetJointController()->SetVelocityPID(
            this->joint->GetScopedName(), this->pid);

        double velocity = 0;
        if (_sdf->HasElement("velocity"))
        {
            velocity = _sdf->Get<double>("velocity");
        }

        this->model->GetJointController()->SetVelocityTarget(
            this->joint->GetScopedName(), velocity);

        this->node = transport::NodePtr(new transport::Node());
        #if GAZEBO_MAJOR_VERSION < 8
        this->node->Init(this->model->GetWorld()->GetName());
        #else
        this->node->Init(this->model->GetWorld()->Name());
        #endif

        std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";
        this->sub = this->node->Subscribe(topicName, &VelodynePlugin::OnMsg, this);
    }

    /// \brief Pointer to the model.
  private:
    physics::ModelPtr model;

    /// \brief Pointer to the joint.
  private:
    physics::JointPtr joint;

    /// \brief A PID controller for the joint.
  private:
    common::PID pid;

  private:  //API Code
    transport::NodePtr node;
    transport::SubscriberPtr sub;

    void OnMsg(ConstVector3dPtr &_msg)
    {
        this->SetVelocity(_msg->x());
    }

};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif