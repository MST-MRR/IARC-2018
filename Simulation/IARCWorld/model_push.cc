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
const double ROOMBA_ROTATE_45_TURN_DURATION = 2.150 / 4 ;
const double ROOMBA_ROTATE_NOISE_MAX_DURATION = 0.238888889 ;

// 2×π×0.330m/s÷(2×π×(0.033))
// 330m/s is speed from IARC7GroundRobot.ino
// 0.033 is radius of wheel
const double ROOBA_WHEEL_SPEED = 0.330 / 0.033 ;
const double SPEED_MULTIPLIER = 1 ;

const unsigned int MOVEMENT_STATE_FORWARD = 0 ;
const unsigned int MOVEMENT_STATE_180_ROTATE = 1 ;
const unsigned int MOVEMENT_STATE_NOISE_ROTATE = 2 ;
const unsigned int MOVEMENT_STATE_45_ROTATE = 3 ;

const bool ROTATE_RIGHT = true ;
const bool ROTATE_LEFT = false ;

namespace gazebo
{

  struct RoombaState
  {
    common::Time MovementState ;
    common::Time Last180RotateStartTime ;
    common::Time Last45RotateStartTime ;
    common::Time LastNoiseRotateStartTime ;
    common::Time NoiseRotateFrames ;
    bool NoiseRotateDirection ;
  } ;



  common::Time GetCurrentTime ( )
  {
    physics::WorldPtr world = physics::get_world("default");
    common::Time CurrentTime = world->SimTime();
    return CurrentTime ;
  }



  void SetStartMatchRoombaState ( RoombaState & MyState )
  {
    common::Time CurrentTime = GetCurrentTime ( ) ;
    MyState.MovementState = MOVEMENT_STATE_FORWARD ;
    MyState.Last180RotateStartTime = CurrentTime ;
    MyState.LastNoiseRotateStartTime = CurrentTime ;
    MyState.NoiseRotateFrames = 0 ;
  }

  RoombaState & SetRotate180State ( RoombaState & MyState )
  {
    common::Time CurrentTime = GetCurrentTime ( ) ;
    MyState.Last180RotateStartTime = CurrentTime ;
    MyState.MovementState = MOVEMENT_STATE_180_ROTATE ;
    return MyState ;
  }
  
  RoombaState & SetRotate45State ( RoombaState & MyState )
  {
    common::Time CurrentTime = GetCurrentTime ( ) ;
    MyState.Last45RotateStartTime = CurrentTime ;
    MyState.MovementState = MOVEMENT_STATE_45_ROTATE ;
    return MyState ;
  }
  
  RoombaState & SetRotateNoiseState ( RoombaState & MyState )
  {
    common::Time CurrentTime = GetCurrentTime ( ) ;
    MyState.LastNoiseRotateStartTime = CurrentTime ;
    MyState.NoiseRotateFrames = ( ((double)rand()) / RAND_MAX * ROOMBA_ROTATE_NOISE_MAX_DURATION ) / SPEED_MULTIPLIER ;
    MyState.NoiseRotateDirection = (bool)round(((double)rand()) / RAND_MAX) ;
    MyState.MovementState = MOVEMENT_STATE_NOISE_ROTATE ;
    return MyState ;
  }
  
  RoombaState & SetForwardState ( RoombaState & MyState )
  {
    MyState.MovementState = MOVEMENT_STATE_FORWARD ;
    return MyState ;
  }
  
  class ModelPush : public ModelPlugin
  {
    private: RoombaState MyState ;
    
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      srand(time(NULL));
      
      // Store the pointer to the model
      this->model = _parent;
      
      //this->model->SetAngularVel(math::Vector3(0, 0, -3));

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelPush::OnUpdate, this, _1));
          
      InitializeSensors ( ) ;
      
      SetStartMatchRoombaState ( MyState ) ;
    }
    
    private: void InitializeSensors ( )
    {
      unsigned int SensorIndex = 0 ;
      while ( SensorIndex < this->model->GetLink("base")->GetSensorCount ( ) )
      {
        std::string SensorName = this->model->GetLink("base")->GetSensorName ( SensorIndex ) ;
        std::cout << SensorName << std::endl;
        sensors::SensorPtr sensor = sensors::get_sensor(std::string(SensorName));
        sensors::ContactSensorPtr contactSensor =
              std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);
        if (contactSensor)
        {
          contactSensor->SetActive(true);
        }
        SensorIndex = SensorIndex + 1 ;
      }
    }
    
    private: void SetForwardMotorPowers ( )
    {
      this->model->GetJoint("right_wheel")->SetVelocity(0, ROOBA_WHEEL_SPEED * SPEED_MULTIPLIER);
      this->model->GetJoint("left_wheel")->SetVelocity(0, ROOBA_WHEEL_SPEED * SPEED_MULTIPLIER);
    }
    
    private: void SetRightTurnMotorPowers ( )
    {
      this->model->GetJoint("right_wheel")->SetVelocity(0, ( -ROOBA_WHEEL_SPEED / 2 ) * SPEED_MULTIPLIER);
      this->model->GetJoint("left_wheel")->SetVelocity(0, ( ROOBA_WHEEL_SPEED / 2 ) * SPEED_MULTIPLIER);
    }
    
    private: void SetNoiseTurnMotorPowers ( RoombaState & MyState )
    {
      if ( MyState.NoiseRotateDirection == ROTATE_RIGHT )
      {
        this->model->GetJoint("right_wheel")->SetVelocity(0, ( -ROOBA_WHEEL_SPEED / 2 ) * SPEED_MULTIPLIER);
        this->model->GetJoint("left_wheel")->SetVelocity(0, ( ROOBA_WHEEL_SPEED / 2 ) * SPEED_MULTIPLIER);
      }
      else
      {
        this->model->GetJoint("right_wheel")->SetVelocity(0, ( ROOBA_WHEEL_SPEED / 2 ) * SPEED_MULTIPLIER);
        this->model->GetJoint("left_wheel")->SetVelocity(0, ( -ROOBA_WHEEL_SPEED / 2 ) * SPEED_MULTIPLIER);
      }
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
    
    bool GetTopTouchSensorState ( )
    {
      //std::cout << "Getting touch sensor state" << std::endl;
      bool Output = false ;
      if ( this->model->GetLink("base")->GetSensorCount ( ) > 1 )
      {
        std::string SensorName = this->model->GetLink("base")->GetSensorName ( 1 ) ;
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
      //std::cout << " cur_time = " << cur_time << std::endl;
      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(math::Vector3(3, 0, 0));
      //this->model->SetAngularVel(math::Vector3(3, 0, 0));
      
      if ( MyState.MovementState == MOVEMENT_STATE_FORWARD )
      {
        SetForwardMotorPowers ( ) ;
        
        common::Time NextRotate180Time = MyState.Last180RotateStartTime + ROOMBA_TIME_BETWEEN_180 / SPEED_MULTIPLIER ;
        if ( GetCurrentTime ( ) > NextRotate180Time )
        {
          MyState = SetRotate180State ( MyState ) ;
        }
        
        common::Time NextRotateNoiseTime = MyState.LastNoiseRotateStartTime + ROOMBA_TIME_BETWEEN_NOISE / SPEED_MULTIPLIER ;
        if ( GetCurrentTime ( ) > NextRotateNoiseTime )
        {
          MyState = SetRotateNoiseState ( MyState ) ;
        }
        
        if ( GetFrontTouchSensorState ( ) == true )
        {
          MyState = SetRotate180State ( MyState ) ;
        }
        if ( GetTopTouchSensorState ( ) == true )
        {
          MyState = SetRotate45State ( MyState ) ;
        }
        
        
      }
      if ( MyState.MovementState == MOVEMENT_STATE_180_ROTATE )
      {
        SetRightTurnMotorPowers ( ) ;
        
        common::Time Rotate180Duration = ( ROOMBA_ROTATE_180_TURN_DURATION / SPEED_MULTIPLIER ) ;
        common::Time EndRotateTime = MyState.Last180RotateStartTime + Rotate180Duration;
        if ( GetCurrentTime ( ) > EndRotateTime )
        {
          MyState = SetForwardState ( MyState ) ;
        }
      }
      if ( MyState.MovementState == MOVEMENT_STATE_45_ROTATE )
      {
        SetRightTurnMotorPowers ( ) ;
        
        common::Time Rotate45Duration = ( ROOMBA_ROTATE_45_TURN_DURATION / SPEED_MULTIPLIER ) ;
        common::Time EndRotateTime = MyState.Last45RotateStartTime + Rotate45Duration;
        if ( GetCurrentTime ( ) > EndRotateTime )
        {
          MyState = SetForwardState ( MyState ) ;
        }
      }
      if ( MyState.MovementState == MOVEMENT_STATE_NOISE_ROTATE )
      {
        SetNoiseTurnMotorPowers ( MyState ) ;
        
        common::Time RotateNoiseDuration = MyState.NoiseRotateFrames ;
        common::Time EndRotateTime = MyState.LastNoiseRotateStartTime + RotateNoiseDuration;
        if ( GetCurrentTime ( ) > EndRotateTime )
        {
          MyState = SetForwardState ( MyState ) ;
        }
        
        if ( GetFrontTouchSensorState ( ) == true )
        {
          MyState = SetRotate180State ( MyState ) ;
        }
      }
        
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
