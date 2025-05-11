#!/usr/bin/env python3

import numpy as np
import sys
import rospy
import cv2
import matplotlib.pyplot as plt

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from bob_ros.msg import path_coordinates
from bob_ros.msg import path_coordinate
from geometry_msgs.msg import Pose

import video_cam


class WaypointFinder():

    def __init__(self):

        rospy.init_node('find_waypoints')

        # the input image has already been Canny edged
        rospy.Subscriber("/image_detection/image", Image, self.imageSubscriberCallback, queue_size=10)
        self.pub = rospy.Publisher("waypoints", path_coordinates, queue_size=1)
        #self.coordinates = coordinates
        self.coordinates = path_coordinates()

        #self.timer = rospy.Timer(rospy.Duration(1), self.publishFunction)

        self.colors = [(255, 255, 255), (255, 0, 0)]

        self.edge_img = None

        self.canvas = None 
        self.width = None
        self.height = None      

        self.points = []
        self.ordered_points = []

        self.scale = 5e-4 * 2        # each pixel is this many meters (0.5mm)

    def sort_for_drawing(self, points):
        """
        Sorts an array of 2D points representing lines in a drawing for a robot to redraw.
        
        Args:
            points: A list of tuples, where each tuple represents a 2D point (x, y).

        Returns:
            A list of tuples representing the points in order for the robot to draw.
        """

        # Define a function to calculate distance
        def distance_to(p, prev):
            dx = p[2] - prev[2]         # coords are stored in  label,y,x  format
            dy = p[1] - prev[1]         # p[0] is actually the label.
            return dx**2 + dy**2  # Euclidean distance squared
        
        # Check if there are any points
        if not points:
            return []

        # Iteratively select closest unvisited point
        start = min(points)     # Select a starting point
        ordered_points = [start]
        prev_point = start
        remaining = points.copy()   # keep track of all unvisited points
        ordered_points = [start]
        while remaining:
            closest = min(remaining, key=lambda p: distance_to(p, prev_point))
            remaining.remove(closest)
            ordered_points.append((closest[0], closest[1], closest[2]))
            prev_point = closest
        
        ordered_points.remove(start)
        '''
        # remove half the points to simplify
        output = []
        for i in range(len(ordered_points)):
            if(i%2 == 0):
                output.append(ordered_points[i])
        return output
        '''
        return ordered_points


    def publishFunction(self):
        self.pub.publish(self.coordinates)

    def imageSubscriberCallback(self, msg: Image) -> None:
        if msg is None:
            rospy.logerr("no Image received.")
            return
    
        bridge = CvBridge()
        edge_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
    
        self.canvas = np.zeros_like(edge_img, np.uint8)   # used to visualise the coloured pixels for debugging
        self.width = np.shape(edge_img)[1]  # get image width and height
        self.height = np.shape(edge_img)[0]

        ## Find connected edges
        total_labels, labels = cv2.connectedComponents(edge_img)   # find which pixels are connected together
        # total_labels is number of connected objects
        # labels is a matrix of same dimensions as original image. Each element of the matrix is given a value (between 0 and total_labels-1) which defines which object that corresponding pixel belongs to.
        
        # collect all pixels part of an edge 
        for y in range(self.height-1):
            for x in range(self.width-1):
                for i in range(1, total_labels):   # label 0 is the background (not part of an edge) hence we start from 1.
                    if labels[y][x] == i:
                        self.points.append((labels[y][x],y,x))
                        # self.canvas[y][x] = self.colors[i%len(self.colors)]    # color in the pixels so we can see them.

        ordered_points = self.sort_for_drawing(self.points.copy())    # sort the points/pixels so they can be used as waypoints.

        self.coordinates = path_coordinates()

        for point in ordered_points:
            coordinate = path_coordinate()

            coordinate.pose.position.x = -point[2]*self.scale
            coordinate.pose.position.y = point[1]*self.scale
            coordinate.pose.position.z = 0.000  # -0.009

            coordinate.pose.orientation.x = 0
            coordinate.pose.orientation.y = 1
            coordinate.pose.orientation.z = 0
            coordinate.pose.orientation.w = 0

            coordinate.edge_label = point[0]

            self.coordinates.poses.append(coordinate)

        cv2.imwrite("edged.png", edge_img)    # Display the edged pixels.

        print("Publishing points")

        self.publishFunction()

                # Display the ordered points with lines inbetween each one 
        # so the path can be seen
        # plt.figure()
        # for i in range(1, len(self.ordered_points)):
        #     p1 = self.ordered_points[i-1]
        #     p2 = self.ordered_points[i]
        #     plt.plot([p1[1], p2[1]], [p1[2], p2[2]], color='red', linewidth=2)

        # plt.title("Ordered Waypoints")
        # plt.show()


def main():

    waypointPublisher = WaypointFinder()

    msg = rospy.wait_for_message("waypoints", path_coordinates, timeout=None)

    rospy.spin()





if __name__ == '__main__':
  main()
