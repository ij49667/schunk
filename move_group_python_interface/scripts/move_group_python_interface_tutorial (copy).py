#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
## END_SUB_TUTORIAL

from geometry_msgs.msg import Point
from std_msgs.msg import String

def joystick_callback(self, msg):

  pose_target = self.move_group.get_current_pose().pose
  pose_target.position.x += 0.2
 
  waypoints = []
  wpose = geometry_msgs.msg.Point()
  wpose.orientation.w = 1.0
  wpose.position.x = waypoints[0].position.x + 0.1
  wpose.position.y = waypoints[0].position.y
  wpose.position.z = waypoints[0].position.z
  waypoints.append(copy.deepcopy(wpose))
  
  wpose.position.z -= 0.10
  waypoints.append(copy.deepcopy(wpose))

  wpose.position.y += 0.05
  waypoints.append(copy.deepcopy(wpose))
  
  (plan, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
                               
  

def listener():

  class MoveGroupPythonInterfaceTutorial(object):
    def __init__(self):
      super(MoveGroupPythonInterfaceTutorial, self).__init__() 
      
      moveit_commander.roscpp_initialize(sys.argv)
      rospy.init_node('move_group_python_interface_node',anonymous=True)
  
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group_name  = "schunk_arm"
  group = moveit_commander.MoveGroupCommander(group_name)
  sub = rospy.Subscriber('/joystick_dref', Point, joystick_callback)
  
  self.robot = robot
  self.scene = scene
  self.group = group
  self.planning_frame = planning_frame
  self.eef_link = eef_link
  self.group_names = group_names
  
  self.sub = sub
  self.joystick_callback = joystick_callback
  
	
  
  rospy.spin()
  

    
  """

  ## You can ask RVIZ to visualize a plan (aka trajectory) for    you.  But the
  ## group.plan() method does this automatically so this is not   that useful
  ## here (it just displays the same trajectory again).
  #print "============ Visualizing plan1"
  ##display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  #display_trajectory.trajectory_start = robot.get_current_state()
 	#display_trajectory.trajectory.append(plan1)
  #display_trajectory_publisher.publish(display_trajectory);

  #print "============ Waiting while plan1 is visualized  (again)..."
  #rospy.sleep(5)


  ## Moving to a pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^
  ##
  ## Moving to a pose goal is similar to the step above
  ## except we now use the go() function. Note that
  ## the pose goal we had set earlier is still active 
  ## and so the robot will try to move to that goal. We will
  ## not use that function in this tutorial since it is 
  ## a blocking function and requires a controller to be active
  ## and report success on execution of a trajectory.

  # Uncomment below line when working with a real robot
  # group.go(wait=True)
  
  # Use execute instead if you would like the robot to follow 
  # the plan that has already been computed
 
  """
  """
  ## Cartesian Paths
  ## ^^^^^^^^^^^^^^^
  ## You can plan a cartesian path directly by specifying a list of waypoints 
  ## for the end-effector to go through.
  waypoints = []

  # start with the current pose
  waypoints.append(group.get_current_pose().pose)
  

  print  group.get_current_pose().pose
  
  # first orient gripper and move forward (+x)
  wpose = geometry_msgs.msg.Pose()
  wpose.orientation.w = 1.0
  wpose.position.x = waypoints[0].position.x + 0.1
  wpose.position.y = waypoints[0].position.y
  wpose.position.z = waypoints[0].position.z
  waypoints.append(copy.deepcopy(wpose))

  print wpose.position.x, wpose.position.y, wpose.position.z

  # second move down
  wpose.position.z -= 0.10
  waypoints.append(copy.deepcopy(wpose))
 
  print wpose.position.x, wpose.position.y, wpose.position.z

  # third move to the side
  wpose.position.y += 0.05
  waypoints.append(copy.deepcopy(wpose))

  print wpose.position.x, wpose.position.y, wpose.position.z

  ## We want the cartesian path to be interpolated at a resolution of 1 cm
  ## which is why we will specify 0.01 as the eef_step in cartesian
  ## translation.  We will specify the jump threshold as 0.0, effectively
  ## disabling it.
  (plan, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
                               

  # Uncomment the line below to execute this plan on a real robot.
  #group.execute(plan)
  print "========== Waiting while RVIZ displays plan3..."
 
  rospy.sleep(5)
  ## Adding/Removing Objects and Attaching/Detaching Objects
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ## First, we will define the collision object message
  #collision_object = moveit_msgs.msg.CollisionObject()
 
  """
	
if __name__=='__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
  
  
