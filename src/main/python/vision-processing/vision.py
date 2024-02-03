"""
THIS IS NOT TO BE DEPLOYED WITH THE ROBOT, IT IS ONLY INCLUDED IN THE REPOSITORY
THIS CODE WILL BE RUN ON THE RASPBERRY PI
"""

# Import Libraries
from wpimath.geometry import *
import cv2
import numpy as np

# Import Methods
from frc_apriltags import USBCamera
from frc_apriltags import startNetworkComms
from frc_apriltags import Streaming

# Defines the camera resolution (width x height)
camRes = (1080, 720)

# Defines the preferred FPS
fps = 60

# initializes usb cameras on the raspberry pi
camera0 = USBCamera(camNum = 0, camPath =   "insertpathhere", resolution = camRes, fps = fps)
camera1 = USBCamera(camNum = 1, camPath = "insertpathhere", resolution = camRes, fps = fps)

# Creates a camera for the drivers
driverCam = Streaming(camNum = 0, resolution = camRes, fps = fps)

# Prealocate space for the detection stream
stream = driverCam.prealocateSpace()

# Starts the network communications
startNetworkComms(5480)

# Main loop
while (True):
    # Get and display the stream
    camera0.displayStream(streamType = 0)
    #camera1.displayStream(streamType = 1)

    # Press q to end the program
    if (camera0.getEnd() == True or camera1.getEnd()):
        break





# Main loop
while (True):
    # Gets the stream
    stream = driverCam.getStream()

    # OpenCV processing here...
    #

    # Sends the stream back
    driverCam.streamImage(stream = stream)

    # Press q to end the program
    if ( driverCam.getEnd() == True ):
        break