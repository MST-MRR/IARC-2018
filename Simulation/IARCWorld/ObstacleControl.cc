#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/sensors/sensors.hh>

const double ROOMBA_TIME_BETWEEN_180 = 20 ;
const double ROOMBA_TIME_BETWEEN_NOISE = 5 ;

const double ROOMBA_ROTATE_NOISE_DEGREES = 5 ;

const double ROOMBA_ROTATE_180_TURN_DURATION = 2.150 ;
const double ROOMBA_ROTATE_NOISE_MAX_DURATION = 0.238888889 ;

// 2×π×0.330m/s÷(2×π×(0.033))
// 330m/s from IARC7GroundRobot.ino
const double ROOBA_RIGHT_WHEEL_SPEED = ( 0.330 + 0.0087 ) / 0.033 ;
const double ROOBA_LEFT_WHEEL_SPEED = ( 0.330 - 0.0087 ) / 0.033 ;
const double SPEED_MULTIPLIER = 1 ;

const unsigned int MOVEMENT_STATE_FORWARD = 0 ;
const unsigned int MOVEMENT_STATE_180_ROTATE = 1 ;
const unsigned int MOVEMENT_STATE_NOISE_ROTATE = 2 ;

const bool ROTATE_RIGHT = true ;
const bool ROTATE_LEFT = false ;

namespace gazebo
{
  
  class ObstacleControl : public ModelPlugin
  {
    private: common::Time LastBumpTime ;
    private: bool IsWaiting ;
    
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      srand(time(NULL));
      
      // Store the pointer to the model
      this->model = _parent;
      
      //this->model->SetAngularVel(math::Vector3(0, 0, -3));

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ObstacleControl::OnUpdate, this, _1));
      
      if ( this->model->GetLink("base")->GetSensorCount ( ) > 0 )
      {
        std::string SensorName = this->model->GetLink("base")->GetSensorName ( 0 ) ;
        std::cout << SensorName << std::endl;
        sensors::SensorPtr sensor = sensors::get_sensor(std::string(SensorName));
        sensors::ContactSensorPtr contactSensor =
              std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);
        if (contactSensor)
        {
          contactSensor->SetActive(true);
        }
      }
      
      IsWaiting = false ;
      physics::WorldPtr world = physics::get_world("default");
      LastBumpTime = world->SimTime();
    }
    
    bool GetFrontTouchSensorState ( )
    {
      //std::cout << "Getting touch sensor state" << std::endl;
      bool Output = false ;
      if ( this->model->GetLink("base")->GetSensorCount ( ) > 0 )
      {
        std::string SensorName = this->model->GetLink("base")->GetSensorName ( 0 ) ;
        sensors::SensorPtr sensor = sensors::get_sensor(std::string(SensorName));
        sensors::ContactSensorPtr contactSensor =
              std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);
        if (contactSensor)
        {
          msgs::Contacts contacts;
          contacts = contactSensor->Contacts();
          if ( contacts.contact_size() > 0 )
          {
            Output = true ;
          }
        }
      }
      return Output ;
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      
      physics::WorldPtr world = physics::get_world("default");
      common::Time cur_time = world->SimTime();
      
      if ( IsWaiting == true )
      {
        this->model->GetJoint("right_wheel")->SetVelocity(0, 0);
        this->model->GetJoint("left_wheel")->SetVelocity(0, 0);
        if ( cur_time > LastBumpTime + 1 )
        {
          IsWaiting = false ;
        }
      }
      else
      {
      
        if ( GetFrontTouchSensorState ( ) == false )
        {
          this->model->GetJoint("right_wheel")->SetVelocity(0, ROOBA_RIGHT_WHEEL_SPEED * SPEED_MULTIPLIER );
          this->model->GetJoint("left_wheel")->SetVelocity(0, ROOBA_LEFT_WHEEL_SPEED * SPEED_MULTIPLIER );
        }
        else
        {
          IsWaiting = true ;
          LastBumpTime = cur_time ;
          this->model->GetJoint("right_wheel")->SetVelocity(0, 0);
          this->model->GetJoint("left_wheel")->SetVelocity(0, 0);
        }
      }
        
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ObstacleControl) ;
}
