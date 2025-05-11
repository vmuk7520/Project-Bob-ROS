#!/usr/bin/env python3

import numpy as np
import sys
import rospy
import cv2
import myutils
import matplotlib.pyplot as plt

from bob_ros.msg import path_coordinates
from bob_ros.msg import path_coordinate
from geometry_msgs.msg import Pose

import video_cam

class WaypointPublisher():

    def __init__(self):
        colors = [(255, 255, 255), (255, 0, 0)]

        # the input image we are trying to draw
        # img = cv2.imread("old_test.png")
        # img = cv2.imread("test.png")
        # img = cv2.imread("jack.jpeg")
        # img = cv2.imread("khit.jpg")
        # img = cv2.imread("jesse.jpeg")
        # img = cv2.imread("test.jpg")
        img = cv2.imread("jenny_tiny.jpg")
        # img = video_cam.getDrawing()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)        # make it grayscale
        gray = cv2.medianBlur(img, 3)                       # blur the image
        edged = cv2.Canny(gray, 50, 200)                    # use edge detector to highlight edge pixels

        canvas = np.zeros_like(img, np.uint8)   # used to visualise the coloured pixels for debugging
        width = len(canvas[0])  # get image width and height
        height = len(canvas)

        ## Find connected edges
        total_labels, labels = cv2.connectedComponents(edged)   # find which pixels are connected together
        # total_labels is number of connected objects
        # labels is a matrix of same dimensions as original image. Each element of the matrix is given a value (between 0 and total_labels-1) which defines which object that corresponding pixel belongs to.

        scale = 5e-4 * 7

        # collect all pixels part of an edge 
        points = []
        for y in range(height-1):
            for x in range(width-1):
                for i in range(1, total_labels):   # label 0 is the background (not part of an edge) hence we start from 1.
                    if labels[y][x] == i:
                        points.append((labels[y][x],y*scale,x*scale))
                        canvas[y][x] = colors[i%len(colors)]    # color in the pixels so we can see them.

        ordered_points = self.sort_for_drawing(points.copy())    # sort the points/pixels so they can be used as waypoints.

        # ordered_points = [[0, 0, 0], [0, 0.01, -0.01], [0, 0.01, -0.02]]

        coordinates = path_coordinates()

        for point in ordered_points:
            coordinate = path_coordinate()

            coordinate.pose.position.x = -point[2]
            coordinate.pose.position.y = point[1]
            coordinate.pose.position.z = 0 + 0.001

            coordinate.pose.orientation.x = 0
            coordinate.pose.orientation.y = 1
            coordinate.pose.orientation.z = 0
            coordinate.pose.orientation.w = 0

            coordinate.edge_label = point[0]

            coordinates.poses.append(coordinate)

        rospy.init_node('find_waypoints')

        self.pub = rospy.Publisher("waypoints", path_coordinates, queue_size=1)
        self.coordinates = coordinates

        self.timer = rospy.Timer(rospy.Duration(1), self.publishFunction)

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
        # remove half the points to simplify
        output = []
        for i in range(len(ordered_points)):
            if(i%2 == 0):
                output.append(ordered_points[i])
        print(len(ordered_points))
        print(len(output))
        return output
        # return ordered_points


    def publishFunction(self, timer):
        self.pub.publish(self.coordinates)


def main():
    # store the coordinates of each ordered point
    # f = open("pathCoordinates.txt", 'w')
    # for point in ordered_points:
    #     f.write(f"{point[0]},{point[1]},{point[2]}\n")    # write these locations out to file
    # f.close()
    # the text file is of this format:
    # label,y,x\n
    # label,y,x\n
    # ...

    # cv2.imwrite("edged.png", canvas)    # Display the edged pixels.

    # Display the ordered points with lines inbetween each one 
    # so the path can be seen
    # plt.figure()
    # for i in range(1, len(ordered_points)):
    #     p1 = ordered_points[i-1]
    #     p2 = ordered_points[i]
    #     plt.plot([p1[1], p2[1]], [p1[2], p2[2]], color='red', linewidth=2)

    # plt.title("Ordered Waypoints")
    # plt.show()



    #### Start publishing to ROS ####

    # intialise ROSnode.

    # pub.publish(coordinates)
    # print("published")


    waypointPublisher = WaypointPublisher()

    msg = rospy.wait_for_message("waypoints", path_coordinates, timeout=None)

    # rospy.spin()





if __name__ == '__main__':
  main()
