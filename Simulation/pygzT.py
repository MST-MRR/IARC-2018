import trollius
from trollius import From

import pygazebo
import pygazebo.msg.joint_cmd_pb2
import sys

FRAME_RATE = 5.0; # Frames Per Second

FORCE_STANDARD = 0.05
TURN_SCALAR = 5

def mod( n, d ):
    r = n
    while r >= d:
        r -= d
    return r

@trollius.coroutine
def publish_loop():
    manager = yield From(pygazebo.connect())

    publisher = yield From(
        manager.advertise('/gazebo/default/create/joint_cmd',
                          'gazebo.msgs.JointCmd'))

    
    msg_coast_L = pygazebo.msg.joint_cmd_pb2.JointCmd()
    msg_coast_L.name = 'create::left_wheel'
    msg_coast_L.axis = 0
    msg_coast_L.force = 0.0
    msg_coast_R = pygazebo.msg.joint_cmd_pb2.JointCmd()
    msg_coast_R.name = 'create::right_wheel'
    msg_coast_R.axis = 0
    msg_coast_R.force = 0.0
    
    msg_accel_forward_L = pygazebo.msg.joint_cmd_pb2.JointCmd()
    msg_accel_forward_L.name = 'create::left_wheel'
    msg_accel_forward_L.axis = 0
    msg_accel_forward_L.force = FORCE_STANDARD
    msg_accel_forward_R = pygazebo.msg.joint_cmd_pb2.JointCmd()
    msg_accel_forward_R.name = 'create::right_wheel'
    msg_accel_forward_R.axis = 0
    msg_accel_forward_R.force = FORCE_STANDARD
    
    msg_accel_backward_L = pygazebo.msg.joint_cmd_pb2.JointCmd()
    msg_accel_backward_L.name = 'create::left_wheel'
    msg_accel_backward_L.axis = 0
    msg_accel_backward_L.force = -FORCE_STANDARD
    msg_accel_backward_R = pygazebo.msg.joint_cmd_pb2.JointCmd()
    msg_accel_backward_R.name = 'create::right_wheel'
    msg_accel_backward_R.axis = 0
    msg_accel_backward_R.force = -FORCE_STANDARD
    
    msg_accel_right_L = pygazebo.msg.joint_cmd_pb2.JointCmd()
    msg_accel_right_L.name = 'create::left_wheel'
    msg_accel_right_L.axis = 0
    msg_accel_right_L.force = FORCE_STANDARD * TURN_SCALAR
    msg_accel_right_R = pygazebo.msg.joint_cmd_pb2.JointCmd()
    msg_accel_right_R.name = 'create::right_wheel'
    msg_accel_right_R.axis = 0
    msg_accel_right_R.force = -FORCE_STANDARD * TURN_SCALAR
    
    msg_accel_left_L = pygazebo.msg.joint_cmd_pb2.JointCmd()
    msg_accel_left_L.name = 'create::left_wheel'
    msg_accel_left_L.axis = 0
    msg_accel_left_L.force = -FORCE_STANDARD * TURN_SCALAR
    msg_accel_left_R = pygazebo.msg.joint_cmd_pb2.JointCmd()
    msg_accel_left_R.name = 'create::right_wheel'
    msg_accel_left_R.axis = 0
    msg_accel_left_R.force = FORCE_STANDARD * TURN_SCALAR
    
    
    
    frame_count = 0

    while True:
      
        if( mod( frame_count, (20 * FRAME_RATE) ) == 0 * FRAME_RATE ):
            yield From(publisher.publish(msg_coast_L))
            yield From(publisher.publish(msg_coast_R))
        elif( mod( frame_count, (20 * FRAME_RATE) ) == 16 * FRAME_RATE ):
            yield From(publisher.publish(msg_accel_backward_L))
            yield From(publisher.publish(msg_accel_backward_R))
        elif( mod( frame_count, (20 * FRAME_RATE) ) == 17 * FRAME_RATE ):
            yield From(publisher.publish(msg_accel_right_L))
            yield From(publisher.publish(msg_accel_right_R))
        elif( mod( frame_count, (20 * FRAME_RATE) ) == 18 * FRAME_RATE ):
            yield From(publisher.publish(msg_accel_left_L))
            yield From(publisher.publish(msg_accel_left_R))
        elif( mod( frame_count, (20 * FRAME_RATE) ) == 19 * FRAME_RATE ):
            yield From(publisher.publish(msg_accel_forward_L))
            yield From(publisher.publish(msg_accel_forward_R))
        yield From(trollius.sleep(1/FRAME_RATE))
        frame_count += 1


loop = trollius.get_event_loop()
loop.run_until_complete(publish_loop())
