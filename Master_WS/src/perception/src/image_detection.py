#!/usr/bin/env python3

"""Node for detecting drawings on a white background and processing them 
   by edge detection.

By default subscribes to '/usb_cam/image_raw/compressed'
By default publishes to '/image_detection/image'

Author: Terry Wang
Date Modified: 13/05/2024
"""

### ROS imports
import rospy
from sensor_msgs.msg import CompressedImage, Image
### Other dependencies
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import math

### The ImageDetection class is a class for detecting a drawing on a white background. The class is able to isolate
### the "canvas" (white background) and then finds the drawing points on that canvas by the applications of edge detection.
class ImageDetection:
    """ImageDetection node which extends the standard ROS Node class."""

    # Topic variables
    topicImagePub = '/image_detection/image'
    topicImageSub = '/usb_cam/image_raw/compressed'

    def __init__(self):
        """IMAGE DETECTION node default constructor."""
        rospy.init_node('ImageDetection')

        # Debugging log
        rospy.loginfo("IMAGE DETECTION NODE INIT")

        # Initialize CvBridge
        self.bridge = CvBridge()


        # Initialize publishers
        self.imagePub = rospy.Publisher(
            ImageDetection.topicImagePub,
            Image,
            queue_size=10
        )

        # Initialize subscribers
        self.imageSub = rospy.Subscriber(
            ImageDetection.topicImageSub,
            CompressedImage,
            self.ImageCallback
        )

        # Initialize empty variables
        self.edges = []
        self.cv_image = []

    """ This callback function processes the incoming CompressedImage messages and displays
        the messages from the live camera feed """
    def ImageCallback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        cv2.imshow("Image Detection", self.cv_image)
        cv2.waitKey(1)

        cv2.setMouseCallback("Image Detection", self.MouseCallback)

    """ This callback function waits for a mousebutton click and calls the ImageDetection functions 
        or publishes the image"""
    def MouseCallback(self, action, x, y, flags, *userdata):
        if action is cv2.EVENT_LBUTTONDOWN:
            self.ImageDetection(self.cv_image)

        if action is cv2.EVENT_RBUTTONDOWN:
            bridge = CvBridge()

            # Publish the image
            self.imagePub.publish(bridge.cv2_to_imgmsg(self.edges, encoding="passthrough"))

            print("Image published")

            # Reset the edged image
            self.edges = []

    def ImageDetection(self, cv_image):
        """ This function takes in an openCV image and isolates a white canvas, which canny edge detection is then applied to"""

        # Convert the image to HSV colourspace
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Convert the image to grayscale
        imgGrayScale = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Blur the grayscaled image
        imgBlurred =  cv2.medianBlur(imgGrayScale, 1) 

        # Set the lower and upper thresholding values
        colorRanges = {
            'white':   (np.array([0, 0, 100]), np.array([180, 120, 255])), 
            'white2':  (np.array([0, 120, 130]), np.array([180, 150, 255])), 
            'white3':  (np.array([0, 0, 100]), np.array([180, 20, 255])), 
        }
    
        # Iterate over each colour
        for color, (lower, upper) in colorRanges.items():

            # Create a mask
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            Contour = None

            # Iterate over contours
            for contour in contours:  

                # Calculate area of Contour
                area = cv2.contourArea(contour)

                # Filter countours by area
                if area > 5000:
                    Contour = contour
                    
                    if Contour is not None:
                        # Calculate aspect ratio
                        x, y, w, h = cv2.boundingRect(Contour)

                        # Draw the bounding box
                        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                        # Extract region of interest from blurred image. Also crop to be 20% smaller.
                        imgROI = imgBlurred[round(y+h*0.1):round(y+h*0.9), round(x+w*0.1):round(x+w*0.9)]   

                        # Perform edge detection
                        self.edges = cv2.Canny(imgROI, 100, 300)

        # Display the image and the mask
        cv2.imshow("imgROI", imgROI)
        cv2.imshow("edges", self.edges)


def main(args=None) -> None:
  """Sits until a callback is triggered using rospy.spin

  Args:
  args (_type_, optional): input arguments. Defaults to None.
  """

  # intialise ROS and the node.
  rospy.init_node('ImageDetection')
  interface = ImageDetection()

  rospy.spin()

  # Destroy the node explicitly
  interface.destroy_node()
  rospy.shutdown()


if __name__ == '__main__':
  main()
