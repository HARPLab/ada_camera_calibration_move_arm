#!/usr/bin/python
"""
This is the base class with some nice template functions for setting up a controller on the ada robot
The move_to_target method should be overwritten in child classes
"""
import rospy
import openravepy
import numpy as np
import time
import adapy
import transforms3d as t3d

import transform_helpers as th

from std_msgs.msg import Float64

distance_to_goal_topic = "/distance_to_goal" # std_msgs/Float64

class AdaControlBase(object):
  def __init__(self, args, endEffName="Mico"): 
    openravepy.RaveInitialize(True)
    openravepy.misc.InitOpenRAVELogging()
    self.env, self.robot = adapy.initialize(
        sim=args.sim,
        attach_viewer=args.viewer,
        env_path=args.env_xml
    )
    # make the robot go at half speed
    vel_limits = self.robot.GetDOFVelocityLimits()
    self.robot.SetDOFVelocityLimits(vel_limits * 0.5)
    self.manip = self.robot.SetActiveManipulator(endEffName)
    self.manip_rob = openravepy.interfaces.BaseManipulation(self.robot) # create the interface for basic manipulation programs

    self.rot = self.generate_target_rotmat()
    self.quat = t3d.quaternions.mat2quat(self.rot)
    
    # even though it's not obvious how we use this, we need to initialize the IKModel on self.robot
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=openravepy.IkParameterization.Type.Transform6D)
    #ikmodel.autogenerate()
    try:
      # ideally this would return 0 instead of erroring if it fails, but for now a try-catch will do the trick
      # actually, a try-catch doesn't help. just run autogenerate whenever you change the robot
      ikmodel.load()
    except:
      ikmodel.autogenerate()

    self.dist_to_goal_publisher = rospy.Publisher(distance_to_goal_topic, Float64, queue_size=10)

  # generate the rotation matrix corresponding to the desired end-effector rotation
  # this multiplying this rotmat times points written in the end-effector frame
  # gives points in the base frame
  def generate_target_rotmat(self):
    rot = np.array([[0,0,1],[0,-1,0],[1,0,0]])
    # the identity position. gripper points along z. back of hand (camera) points along y
    rot = np.eye(3)
    #rot = np.array([[1,0,0],[0,-1,0],[0,0,-1]])
    rot = np.array([[0,0,1],[0,1,0],[-1,0,0]])
    # close gazebo init start position
    rot = np.array([[0,0,1],[1,0,0],[0,1,0]])
    # camera between hand and face, by just rotating joint 6 from gazebo init (rotate around y from identity)
    rot = np.array([[0,0,1],[0,1,0],[-1,0,0]])
    roty = t3d.quaternions.quat2mat([0,np.sqrt(2)/2, 0, np.sqrt(2)/2])
    rotx = t3d.quaternions.quat2mat([-np.sqrt(2)/2,0, 0, np.sqrt(2)/2])
    rot = roty.dot(rotx)
    return rot
  
  # return a boolean for whether the end-effector is already at the endLoc target
  # if the end-effector is already close enough to the target, then there's no need
  # to move the end-effector to the target
  def is_close_enough_to_target(self, endLoc, epsilon = 0.01):
    dist = th.distance(self.get_cur_loc(), endLoc)
    rospy.logwarn("going to %s. current distance to there is %f" % (endLoc, dist))
    return(dist < epsilon)

  def get_cur_loc(self):
    Tee = self.manip.GetEndEffectorTransform()
    curLoc = Tee[0:3,3]
    return(curLoc)

  # for at most timeoutSecs, compute and move toward the input endLoc
  # endLoc must be a length 3 np.array
  # if constrainMotion is set to False, don't allow the robot end effector to rotate, and only allow linear motion toward the goal
  # otherwise, don't constrain motion
  def move_to_target(self, endLoc, timeoutSecs=0, constrainMotion=False):
    '''
    This move_to_target method should be overwritten in child classes
    '''
    raise NotImplementedError("This move_to_target method should be overwritten in child classes")
