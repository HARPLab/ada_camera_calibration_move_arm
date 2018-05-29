#!/usr/bin/python
"""
Use the MoveToHandPosition command to move the hand via cartesian control
"""
import rospy
import numpy as np
import types

import transforms3d as t3d
import transform_helpers as th
from ada_control_base import AdaControlBase

def plug_in_cartesian_control(base_controller):
  # https://stackoverflow.com/questions/972/adding-a-method-to-an-existing-object-instance
  base_controller.move_to_target = types.MethodType(move_to_target, base_controller)
  
def move_to_target(self, endLoc, constrainMotion=False):
  if self.is_close_enough_to_target(endLoc):
    return 
  with self.env:
    Tgoal = np.concatenate((np.concatenate((self.rot, np.transpose([endLoc])), axis=1),[[0,0,0,1]]), axis=0)
    constrainttaskmatrix=np.eye(4)
    constraintmatrix = np.eye(4) 
    constrainterrorthresh = 0.2 # tutorial had 0.005
    constraintfreedoms = [0,0,0,0,0,0] # don't penalize any direction of motion
    if constrainMotion:
      # don't let any x,y,z rotations happen
      # don't let the robot move in the x,y directions in the task frame (only let it move in the z direction)
      constraintfreedoms = [1,1,1,1,0,0]
      motion_frame = th.get_motion_frame_matrix(self.manip.GetTransform(), endLoc)
      constrainttaskmatrix = np.eye(4) 
      constraintmatrix = np.linalg.inv(motion_frame)
    
    traj = self.manip_rob.MoveToHandPosition(matrices=[Tgoal],maxiter=3000,maxtries=10,seedik=40,
      constraintfreedoms=constraintfreedoms,
      constraintmatrix=constraintmatrix, 
      constrainttaskmatrix=constrainttaskmatrix,
      constrainterrorthresh=constrainterrorthresh,
      outputtrajobj=True,
      execute=False,
      steplength=0.002) # tutorial had 0.002
  rospy.logwarn("Executing trajectory")
  self.robot.ExecuteTrajectory(traj)
  rospy.logwarn("Waiting for trajectory")
  self.robot.WaitForController(0)
  rospy.logwarn("Trajectory should be done now")
  curtrans = self.manip.GetEndEffectorTransform()
  #rospy.logwarn("curtrans is %s"%curtrans)
  curquat = t3d.quaternions.mat2quat(curtrans[0:3,:][:,0:3])
  #rospy.logwarn("curquat is %s"%curquat)
  rotjac = self.manip.CalculateRotationJacobian()
  #rospy.logwarn("rotjac is %s"%rotjac)
  angVeljac = self.manip.CalculateAngularVelocityJacobian()
  #rospy.logwarn("angVelJac is %s"%angVeljac)

class AdaCartesianControl(AdaControlBase):
  def __init__(self, args, endEffName = "Mico"):
    super(AdaCartesianControl, self).__init__(args, endEffName)
    plug_in_cartesian_control(self)
