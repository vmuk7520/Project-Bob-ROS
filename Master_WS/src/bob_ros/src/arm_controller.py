#!/usr/bin/env python3
"""
Node for controlling the movement of a UR5e arm by specifying either joint positions or 
a pose goal
"""

import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from bob_ros.msg import path_coordinate
from bob_ros.msg import path_coordinates
from scipy.spatial.transform import Rotation
from set_collision_scene import CollisionScene

import numpy as np

class ArmController():
    """ Class to send control commands to the UR5e arm """

    def __init__(self):
        """ initialise the object """

        self.robot = moveit_commander.RobotCommander()

        # MoveGroupCommander object to plan and execute arm commands
        groupName = "manipulator"
        self.moveGroup = moveit_commander.MoveGroupCommander(groupName)

        # MoveGroupCommander object to build collision scene
        self.scene = moveit_commander.PlanningSceneInterface()

        # waypoints subscriber
        self.sub = rospy.Subscriber('/arm_waypoints', path_coordinates, self.pointsCallBack)

        self.success = True

    def pointsCallBack(self, coordinates):
        print("\n##############BEGIN##################")

        # Intialize empty waypoint array
        waypoints = []

        # Append all recieved poses to waypoints
        for point in coordinates.poses:
                waypoints.append(point.pose)

        print(f'Attempting to move to starting pose:\n{waypoints[0]}')

        self.goToPoseGoal(waypoints[0])
        if self.success == True:
            print("Reached Starting Pose")
        else:
            print("Unable to draw given image due to unreachable pose or disconnected controller\nWaiting for new waypoints...")
            quit()

        print("\nComputing Cartesian Path...")

        # Run iterative computation until complete drawing
        progress = 0
        print(f'Progress completed: {progress}%')
        size = len(waypoints)
        while progress <= 99:
            plan, fraction = self.plan_cartesian_path(waypoints)
            progress += (fraction*(100-progress))
            print(f'Progress completed: {progress:.2f}%')
            self.execute_plan(plan)
            waypoints = waypoints[round(fraction*size):]
            size = len(waypoints)
            
        print("Drawing Finished going home")

        joints = np.radians([90,-90,90,90,90,0])
        self.goToJointState(joints)

        print("##############END##################")

        

    def goToJointState(self, jointGoal):
        """ send the arm to a given set of joint states """
        if(len(jointGoal) != 6):
            print("Incorrect number of joint angles provided")
            return -1

        # send the robot to the given joint positions
        self.moveGroup.go(jointGoal, wait=True)

        # ensure there is no residual movement
        self.moveGroup.stop()

    def goToPoseGoal(self, poseGoal):
        """ send the arm to a given pose """
        self.moveGroup.set_pose_target(poseGoal)

        # send the robot to the given pose
        self.success = self.moveGroup.go(wait=True)

        # ensure there is no residual movement
        self.moveGroup.stop()

        self.moveGroup.clear_pose_targets()

    def plan_cartesian_path(self, waypoints):
        ''' Plan a Cartesian path directly by specifying a list of waypoints
        for the end-effector to go through.'''

        (plan, fraction) = self.moveGroup.compute_cartesian_path(
            waypoints, 0.001, 50  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction
    
    def execute_plan(self, plan):

        move_group = self.moveGroup

        '''execute if you would like the robot to follow
        the plan that has already been computed:'''
        move_group.execute(plan, wait=True)

    def eulerToQuaternion(self, x, y, z, degrees = False):
        """ convert rotation as euler angles to a quaternion """

        rotation = Rotation.from_euler('xyz', [x, y, z], degrees=degrees)

        quaternion = rotation.as_quat()

        quaternionMessage = geometry_msgs.msg.Quaternion()

        quaternionMessage.x = quaternion[0]
        quaternionMessage.y = quaternion[1]
        quaternionMessage.z = quaternion[2]
        quaternionMessage.w = quaternion[3]

        return quaternionMessage

    def quaternionToEuler(self, quaternionMessage):
        quaternion = [quaternionMessage.x, quaternionMessage.y, quaternionMessage.z, quaternionMessage.w]

        rotation = Rotation.from_quat(quaternion)

        eulerAngles = rotation.as_euler('xyz')

        return eulerAngles

def main():
    try:
        rospy.init_node("ArmController")
        
        # intiate CollisionScene object
        load_scene = CollisionScene()

        # Add obstructions
        load_scene.add_collisions()
        load_scene.add_marker()
        load_scene.attach_marker()

        # Initiate ArmController Object
        armController = ArmController()

        # Go to Initial Drawing State
        joints = np.radians([90,-90,90,90,90,0])
        armController.goToJointState(joints)

        print('\nWaiting for Waypoints...')
        rospy.spin()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
