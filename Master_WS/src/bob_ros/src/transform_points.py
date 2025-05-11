#!/usr/bin/env python3
"""
Module to determine where the drawing plane is and convert a set of points from this plane to
coordinates in the arm's reference frame
"""

# python imports
import collections
import os
from copy import deepcopy
import cv2
import numpy as np
from scipy.linalg import lstsq
from scipy.spatial.transform import Rotation
import pyrealsense2.pyrealsense2 as rs2
from cv_bridge import CvBridge, CvBridgeError

# ros imports
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from realsense2_camera.msg import Extrinsics
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
import tf

# custom message imports
from bob_ros.msg import path_coordinate
from bob_ros.msg import path_coordinates

# list length to be used for the rolling average
listLength = 15

# values used for neighbout depth checking
maxNeighbourDistance = 0.0003
neighbourCheckDepth = 6
maxFarNeighbours = 0.6 * (2 * neighbourCheckDepth + 1)**2
maxDepth = 2500

maxIntensity = 150

class PointTransformer:
    """ Class to transform points from the drawing surface to the arm reference frame """
    def __init__(self):
        """ Initialise ROS subscribers and publishers and all relevant variables for the class """
        # booleans to know when to perform certain functions
        self.planeExists = False

        # for OpenCV
        self.bridge = CvBridge()

        # initialise publishers and subscribers
        self.sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.imageDepthCallback)
        self.sub_info = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.imageDepthInfoCallback)
        self.depthIntrinsics = None

        rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.ImageCallback)

        rospy.Subscriber("/waypoints", path_coordinates, self.waypointCallback)
        self.waypointPublisher = rospy.Publisher("arm_waypoints", path_coordinates, queue_size=1)

        self.transformBroadcaster = tf.TransformBroadcaster()

        self.tl = tf.TransformListener()

        self.pointPublisher = rospy.Publisher("visualization_marker", Marker, queue_size = 100)
        self.planePublisher = rospy.Publisher("visualization_marker", Marker, queue_size = 100)

        # variables for the OpenCV mouse interface
        self.latchMouse = False
        self.selectedPixels = []
        self.sentPixels = []

        # for rejecting points that are too bright
        self.intensityMask = None

        # variables for fitting the plane to the surface
        self.planeCoords = []
        self.points = []

        self.planeDepthList = []

        self.planePoseList = collections.deque(maxlen=listLength)

        self.planePose = Pose()

        self.centreX = 0
        self.centreY = 0

        rospy.Timer(rospy.Duration(2), self.timerCallback)

        self.planeCentreImage = None


    def imageDepthCallback(self, data):
        """
        Callback for when depth data is received:
            Read image
            Fit Plane
            Transform points from plane to arm and publish list
        """

        # convert the received image message to opencv
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

        # set variables to initial values
        sumDepth = 0
        totalReadings = 0

        self.planeCoords = []
        self.points = []

        # get the requested image centre from the opencv window and publish as a tf
        if self.planeCentreImage is not None and self.planeExists:
            # convert requested centre to 3D coordinates
            depth = cv_image[self.planeCentreImage[1], self.planeCentreImage[0]]
            coords = rs2.rs2_deproject_pixel_to_point(self.depthIntrinsics, [self.planeCentreImage[0], self.planeCentreImage[1]], depth)

            cameraCoords = PoseStamped()

            cameraCoords.header.seq = 1
            cameraCoords.header.stamp = self.tl.getLatestCommonTime("plane", "camera_depth_optical_frame")
            cameraCoords.header.frame_id = "camera_depth_optical_frame"

            cameraCoords.pose.orientation.x = 0
            cameraCoords.pose.orientation.y = 0
            cameraCoords.pose.orientation.z = 0
            cameraCoords.pose.orientation.w = 1

            cameraCoords.pose.position.x = coords[0]/1000
            cameraCoords.pose.position.y = coords[1]/1000
            cameraCoords.pose.position.z = coords[2]/1000

            # convert 3D coordinates from camera frame to plane frame
            planeCoords = self.tl.transformPose("plane", cameraCoords)

            # publish transform from fitted plane to requested plane centre
            self.transformBroadcaster.sendTransform(
                    (planeCoords.pose.position.x,
                    planeCoords.pose.position.y,
                    planeCoords.pose.position.z),
                    (0, 0, 0, 1),
                    rospy.Time.now(),
                    "plane_centre",
                    "plane")

        # convert the list of pixels to coordinates
        if len(self.sentPixels) > 0:
            if len(self.sentPixels) != len(self.planeDepthList):
                reset = True
            else:
                reset = False

            totalPoints = 0

            xValues = []
            yValues = []
            zValues = []
            depthValues = []

            for (index, pixel) in enumerate(self.sentPixels):
                depth = cv_image[pixel[1], pixel[0]]

                # check if the neighbouring pixels are within the given threshold
                surroundingFar = 0
                for x in range(pixel[0]-neighbourCheckDepth, pixel[0]+neighbourCheckDepth+1):
                    for y in range(pixel[1]-neighbourCheckDepth, pixel[1]+neighbourCheckDepth+1):
                        if abs(depth - cv_image[y, x]) > maxNeighbourDistance:
                            surroundingFar += 1

                # only proceed if enough of the surrounding pixels have a close enough depth
                if(0 < depth < maxDepth and surroundingFar > maxFarNeighbours):
                    sumDepth += depth
                    totalReadings += 1

                    depthValues.append(depth)

                    coords = rs2.rs2_deproject_pixel_to_point(self.depthIntrinsics, [pixel[0], pixel[1]], depth)

                    if(reset):
                        self.planeDepthList.append(collections.deque(maxlen=listLength))

                    self.planeDepthList[index].append(depth)

                    # create the point and append to the list
                    point = Point()

                    point.x = coords[0]/1000
                    point.y = coords[1]/1000
                    point.z = coords[2]/1000

                    totalPoints += 1

                    xValues.append(point.x)
                    yValues.append(point.y)
                    zValues.append(point.z)

                    self.points.append(point)

                    self.planeCoords.append([point.x, point.y, point.z])



            if(totalPoints > 0):
                # only take the points with a depth within the boundary range
                boundary = 0.1
                if len(depthValues) > 0:
                    low = np.quantile(depthValues, boundary)
                    high = np.quantile(depthValues, 1 - boundary)
                    toKeep = []
                    for (i, depth) in enumerate(depthValues):
                        if low < depth < high:
                            toKeep.append(i)

                cleanedPoints = []
                for i in toKeep:
                    cleanedPoints.append(self.points[i])

                self.points = deepcopy(cleanedPoints)

        if len(self.planeCoords) >= 3:
            coordinatesArray = np.array(self.planeCoords)

            # Extract the x and y values, and create the column of 1s
            xValues = coordinatesArray[:, 0]
            yValues = coordinatesArray[:, 1]
            ones = np.ones(xValues.shape)

            # Extract the z values for matrix B
            zValues = coordinatesArray[:, 2]

            # Create matrix A by stacking x, y, and ones columns
            A = np.column_stack((xValues, yValues, ones))

            # Create matrix B by reshaping z_values into a column vector
            B = zValues.reshape(-1, 1)

            # fit the plane using linear least squares
            planeEquation, residual, rnk, s = lstsq(A, B)

            a, b, c = planeEquation

            # calculate the euler angles that the plane is rotated by
            eulerAngles = self.planeToEuler(float(a), float(b), float(c))

            planePose = Pose()

            # set the plane pose as the median value from the list
            averageX = np.median([point.x for point in self.points])
            averageY = np.median([point.y for point in self.points])
            averageZ = np.median([point.z for point in self.points])

            planePose.position.x = averageX
            planePose.position.y = averageY
            planePose.position.z = averageZ

            planePose.orientation = self.eulerToQuaternion(eulerAngles)

            self.planePoseList.append(planePose)

            # set the plane pose based on the median angle about x and y
            angleList = []
            for (index, pose) in enumerate(self.planePoseList):
                if eulerAngles[0] != None:
                    angleList.append(eulerAngles[0]/eulerAngles[1])

            index = np.argsort(angleList)[len(angleList)//2]

            self.planePose = self.planePoseList[index]

            # create the markers for the points
            marker = Marker()
            marker.header.frame_id = "camera_depth_optical_frame"
            marker.ns = "points"

            marker.type = marker.POINTS
            marker.action = marker.ADD
            marker.pose.orientation.w = 1

            marker.points = self.points
            t = rospy.Duration()
            marker.lifetime = t
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0

            # create the marker for the plane
            plane = Marker()
            plane.header.frame_id = "camera_depth_optical_frame"
            plane.ns = "plane"

            plane.type = marker.CUBE
            plane.action = marker.ADD

            plane.pose = self.planePose

            t = rospy.Duration()
            plane.lifetime = t

            plane.scale.x = 1.0
            plane.scale.y = 1.0
            plane.scale.z = 0

            plane.color.r = 0
            plane.color.g = 1
            plane.color.b = 0
            plane.color.a = 1

            # publish the markers and the transform to the plane
            if plane.pose.position.z != None:
                self.planeExists = True
                self.pointPublisher.publish(marker)
                self.planePublisher.publish(plane)

                self.transformBroadcaster.sendTransform(
                        (planePose.position.x,
                        planePose.position.y,
                        planePose.position.z),
                        (planePose.orientation.x,
                        planePose.orientation.y,
                        planePose.orientation.z,
                        planePose.orientation.w),
                        rospy.Time.now(),
                        "plane",
                        "camera_depth_optical_frame")

    def imageDepthInfoCallback(self, cameraInfo):
        """ Get the info about the depth data """
        try:
            if self.depthIntrinsics:
                return
            self.depthIntrinsics = rs2.intrinsics()
            self.depthIntrinsics.width = cameraInfo.width
            self.depthIntrinsics.height = cameraInfo.height
            self.depthIntrinsics.ppx = cameraInfo.K[2]
            self.depthIntrinsics.ppy = cameraInfo.K[5]
            self.depthIntrinsics.fx = cameraInfo.K[0]
            self.depthIntrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.depthIntrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.depthIntrinsics.model = rs2.distortion.kannala_brandt4
            self.depthIntrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

    def ImageCallback(self, msg: CompressedImage) -> None:
        """ Get the colour image and draw the selected dots on it """

        # Debugging Log
        if msg is None:
            rospy.logerr("no Image received.")
            return
        
        # Image pre-processing
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        _, mask = cv2.threshold(img_gray, maxIntensity, 255, cv2.THRESH_BINARY)

        self.intensityMask = mask

        if len(self.selectedPixels) > 0:
            for pixel in self.selectedPixels[:-1]:
                cv2.circle(cv_image, pixel, 3, (0, 255, 0), -1)

            cv2.circle(cv_image, self.selectedPixels[-1], 3, (255, 0, 0), -1)

        cv2.imshow("colour image", cv_image)

        # set the callback for selecting points with the mouse
        cv2.setMouseCallback("colour image", self.selectCanvas)

        cv2.waitKey(1)

    def selectCanvas(self, action, x, y, flags, *userdata):
        """ Callback to handle mouse clicks in the image window """

        # reset and set point centre if right button pressed
        if action is cv2.EVENT_RBUTTONDOWN:
            self.selectedPixels = []
            self.sentPixels = []
            self.planeCentreImage = (x, y)

        # select points while left button is held down
        if action is cv2.EVENT_LBUTTONDOWN:
            self.latchMouse = True
        elif action is cv2.EVENT_LBUTTONUP:
            self.latchMouse = False

        # send the selected pixels if the middle button is pressed
        if action is cv2.EVENT_MBUTTONDOWN:
            self.sentPixels = deepcopy(self.selectedPixels)
            # self.sentPixels = []

            xValues = [pixel[0] for pixel in self.selectedPixels]
            yValues = [pixel[1] for pixel in self.selectedPixels]

            minX = min(xValues)
            maxX = max(xValues)
            minY = min(yValues)
            maxY = max(yValues)

            for xValue in range(minX, maxX):
                for yValue in range(minY, maxY):
                    if yValue % 3 == 0 and xValue % 3 == 0:
                        # only allow pixels to be selected if they aren't too bright
                        if self.intensityMask is not None:
                            if self.intensityMask[yValue, xValue] < 130:
                                self.sentPixels.append((xValue, yValue))

        if(self.latchMouse):
            if self.intensityMask is not None:
                if self.intensityMask[y, x] < 130:
                    self.selectedPixels.append((x, y))

    def planeToEuler(self, a, b, c):
        """ Get the euler angles from a plane of the form ax + by + c = z """

        # Normal vector of the plane
        normalVector = np.array([a, b, -1])
        
        # Normalize the normal vector
        norm = np.linalg.norm(normalVector)
        normalVectorNorm = normalVector / norm

        zAxisVector = np.array([0, 0, 1])

        # calculate the rotation between the normal vecotr and the z axis of the camera
        rotation, _ = Rotation.align_vectors([normalVectorNorm], [zAxisVector])

        # convert to euler angles
        euler = rotation.as_euler('zyx')

        return euler

    def eulerToQuaternion(self, eulerAngles):
        """ Convert from euler angles to a quaternion message """
        rotation = Rotation.from_euler('zyx', eulerAngles)

        quaternion = rotation.as_quat()

        quaternionMessage = Quaternion()

        quaternionMessage.x = quaternion[0]
        quaternionMessage.y = quaternion[1]
        quaternionMessage.z = quaternion[2]
        quaternionMessage.w = quaternion[3]

        return quaternionMessage

    def waypointCallback(self, coordinates):
        """
        Callback to convert waypoints from the plane to the are reference frame
        and publish new list
        """
        if self.planeExists and len(coordinates.poses) > 0:
            baseCoordinates = path_coordinates()

            prevLabel = None

            prevBoardCoords = None

            # iterate through each coordinate
            for coordinate in coordinates.poses:
                baseCoordinate = path_coordinate()

                pose = coordinate.pose
                label = coordinate.edge_label

                # add an offset so the pen lifts off between lines
                if prevLabel is not None and label != prevLabel:
                    prevBoardCoords.pose.position.z += 0.03

                    baseCoords = self.tl.transformPose("base_link", prevBoardCoords)

                    baseCoordinate = path_coordinate()
                    baseCoordinate.edge_label = prevLabel
                    baseCoordinate.pose = baseCoords.pose

                    baseCoordinates.poses.append(baseCoordinate)

                    boardCoords = PoseStamped()

                    boardCoords.header.seq = 1
                    boardCoords.header.stamp = self.tl.getLatestCommonTime("base_link", "plane_centre")
                    boardCoords.header.frame_id = "plane_centre"

                    boardCoords.pose = deepcopy(pose)

                    boardCoords.pose.position.z += 0.275 + 0.03

                    # transform from the plane (board) to the arm base frame
                    baseCoords = self.tl.transformPose("base_link", boardCoords)

                    baseCoordinate = path_coordinate()
                    baseCoordinate.edge_label = label
                    baseCoordinate.pose = baseCoords.pose

                    baseCoordinates.poses.append(baseCoordinate)

                boardCoords = PoseStamped()

                boardCoords.header.seq = 1
                boardCoords.header.stamp = self.tl.getLatestCommonTime("base_link", "plane_centre")
                boardCoords.header.frame_id = "plane_centre"

                boardCoords.pose = pose

                boardCoords.pose.position.z += 0.275

                baseCoords = self.tl.transformPose("base_link", boardCoords)

                baseCoordinate = path_coordinate()
                baseCoordinate.edge_label = label
                baseCoordinate.pose = baseCoords.pose

                baseCoordinates.poses.append(baseCoordinate)

                prevLabel = label
                prevBoardCoords = deepcopy(boardCoords)

            boardCoords = PoseStamped()

            boardCoords.header.seq = 1
            boardCoords.header.stamp = self.tl.getLatestCommonTime("base_link", "plane_centre")
            boardCoords.header.frame_id = "plane_centre"

            boardCoords.pose = deepcopy(coordinates.poses[-1].pose)

            boardCoords.pose.position.z += 0.275 + 0.03

            baseCoords = self.tl.transformPose("base_link", boardCoords)

            baseCoordinate = path_coordinate()
            baseCoordinate.edge_label = coordinates.poses[-1].edge_label
            baseCoordinate.pose = baseCoords.pose

            baseCoordinates.poses.append(baseCoordinate)

            print("publishing base coordinates")
            self.waypointPublisher.publish(baseCoordinates)

    def timerCallback(self, event):
        """ Callback to publish the plane TFs every second """
        if self.planeExists:
            self.transformBroadcaster.sendTransform(
                    (self.planePose.position.x,
                    self.planePose.position.y,
                    self.planePose.position.z),
                    (self.planePose.orientation.x,
                    self.planePose.orientation.y,
                    self.planePose.orientation.z,
                    self.planePose.orientation.w),
                    rospy.Time.now(),
                    "plane",
                    "camera_depth_optical_frame")

            self.transformBroadcaster.sendTransform(
                    (0, 0, -0.001),
                    (0, 0, 0, 1),
                    rospy.Time.now(),
                    "collision_plane",
                    "plane_centre")



def main():
    pointTransformer = PointTransformer()

    rospy.spin()

if __name__ == '__main__':
    rospy.init_node("Transform_Points")
    main()
