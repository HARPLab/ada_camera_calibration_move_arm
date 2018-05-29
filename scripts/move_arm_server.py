#!/usr/bin/env python
import rospy
import argparse
import numpy as np

from ada_camera_calibration_move_arm_msgs.srv import MoveArm, MoveArmResponse
from ada_cartesian_control import AdaCartesianControl

def main(args):
  # initialize the ros node 
  rospy.init_node('move_arm_server', anonymous=True)
  mas = MoveArmService(args)
  s = rospy.Service('move_arm', MoveArm, mas.handle_move_arm)
  rospy.spin() 

class MoveArmService:
  def __init__(self, args):
    self.ada_control = AdaCartesianControl(args, endEffName="Mico")

  # takes in a MoveArm request and calls ada_control 
  # to move the arm based on that request
  def handle_move_arm(self, req):
    # move_to_target's endLoc should be a length 3 np.array
    # of the coordinates to move the end-effector of the arm to in
    # cartesian coordinates relative to the base frame of the arm
    isSuccess = True
    try:
      #rospy.logwarn("Constrain motion is %s"%req.constrainMotion)
      self.ada_control.move_to_target(endLoc=np.array([req.target.x, req.target.y, req.target.z]), constrainMotion=req.constrainMotion)
    except Exception as e:
      rospy.logerr(e)
      isSuccess = False
      #raise
    return MoveArmResponse(isSuccess)

if __name__=="__main__":
  # parse input arguments
  parser = argparse.ArgumentParser(description='service server node to move arm to a given position')
  parser.add_argument('-s', '--sim', action='store_true',
                          help='simulation mode')
  parser.add_argument('-v', '--viewer', nargs='?', const=True,
                          help='attach a viewer of the specified type')
  parser.add_argument('--env-xml', type=str,
                          help='environment XML file; defaults to an empty environment')
  parser.add_argument('--debug', action='store_true',
                          help='enable debug logging')
  args = parser.parse_args(rospy.myargv()[1:])
  main(args) 
 
  
   
  
  
