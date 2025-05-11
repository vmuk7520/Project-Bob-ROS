#!/usr/bin/env python

# Python 2/3 compatibility import
from __future__ import print_function

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
import geometry_msgs.msg
import tf
import time
import sys


class CollisionScene(object):
    def __init__(self):
        self._scene = PlanningSceneInterface()

        # clear the scene
        self._scene.remove_world_object()

        self.robot = RobotCommander()

        self.box_name = "Marker"
        self.eef_link = "tool0"

        self.tl = tf.TransformListener()
        
        # Add oneshot subscriber to read whiteboard transform
        self.timer = rospy.Timer(rospy.Duration(1), self.add_drawing_surface, oneshot = True)

        # pause to wait for rviz to load
        print("============ Waiting while RVIZ displays the scene with obstacles...")

    def add_drawing_surface(self, data):

        self.tl.waitForTransform("base_link", "collision_plane", rospy.Time(), rospy.Duration(10))
        (trans,rot) = self.tl.lookupTransform("base_link", "collision_plane", rospy.Time(0))

        # Define whiiteboard pose using message
        plane_pose = [trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]]
        plane_dimensions = [3, 3, 0.001]

        self.add_box_object("drawing_surface", plane_dimensions, plane_pose)

        print("========== Added Drawing Surface")

    def add_collisions(self):
        
        # Define table and metal beam

        plane1_pose = [0, 0, 0.08, 0, 0, 0, 1]
        plane1_dimensions = [3, 3, 0.001]

        plane2_pose = [0, 0.78, 0, 0, 0, 0, 1]
        plane2_dimensions= [3, 0.001, 3]

        plane2_pose = [0, 1.041, 0, 0, 0, 0, 1]
        plane2_dimensions = [0.15, 0.015, 3]

        plane3_pose = [0, 0.0, 1.254, 0, 0, 0, 1]
        plane3_dimensions = [0.15, 3, 0.015]

        self.add_box_object("table", plane1_dimensions, plane1_pose)
        self.add_box_object("Bar1", plane2_dimensions, plane2_pose)
        self.add_box_object("Bar2", plane3_dimensions, plane3_pose)

        print("========== Added Collision Objects")

    def add_box_object(self, name, dimensions, pose):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "base_link"
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = pose[0]
        p.pose.position.y = pose[1]
        p.pose.position.z = pose[2]
        p.pose.orientation.x = pose[3]
        p.pose.orientation.y = pose[4]
        p.pose.orientation.z = pose[5]
        p.pose.orientation.w = pose[6]

        self._scene.add_box(name, p, (dimensions[0], dimensions[1], dimensions[2]))

    def add_marker(self, timeout=4):

        box_name = 'self.box_name'
        scene = self._scene
        
        # create a box in the planning scene:
        
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "tool0"
        box_pose.pose.orientation.w = 1.0
        marker_offset = 0.265
        box_pose.pose.position.z = marker_offset/2+0.001  # above the end effector frame
        box_name = "Marker"
        scene.add_box(box_name, box_pose, size=(0.07, 0.07, marker_offset))

        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_marker(self, timeout=4):

        box_name = self.box_name
        scene = self._scene
        eef_link = self.eef_link
        
        # Attach the box to the end effector. 
        scene.attach_box(eef_link, box_name)

        print("========== Attached Marker")

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )
    
    def detach_marker(self, timeout=4):

        box_name = self.box_name
        scene = self._scene
        eef_link = self.eef_link
        
        # can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_marker(self, timeout=4):

        box_name = self.box_name
        scene = self._scene

        # remove the box from the world.
        scene.remove_world_object(box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )

    
    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        box_name = self.box_name
        scene = self._scene
        
        # Ensuring Collision Updates Are Received

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False



if __name__ == "__main__":
    rospy.init_node("collision_scene_example_cluttered")
    while (
        not rospy.search_param("robot_description_semantic") and not rospy.is_shutdown()
    ):
        time.sleep(0.5)
    load_scene = CollisionScene()

    load_scene.add_collisions()

    load_scene.add_marker()
    load_scene.attach_marker()

    quit()

    # rospy.spin()

    # load_scene.detach_marker()
    # load_scene.remove_marker()


    
