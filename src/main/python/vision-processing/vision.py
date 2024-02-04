"""
THIS IS NOT TO BE DEPLOYED WITH THE ROBOT, IT IS ONLY INCLUDED IN THE REPOSITORY
THIS CODE WILL BE RUN ON THE RASPBERRY PI
"""

import cv2
import numpy as np
from frc_apriltags import USBCamera

camera = USBCamera(camNum = 0, resolution=(1080, 720), fps=60)

# Main loop
while (True):
    # Get and display the stream
    camera.displayStream(streamType = 0)

    # Press q to end the program
    if ( camera.getEnd() == True ):
        break