"""
THIS IS NOT TO BE DEPLOYED WITH THE ROBOT, IT IS ONLY INCLUDED IN THE REPOSITORY
"""

import cscore
from cscore import CameraServer, CvSink, CvSource, MjpegServer, UsbCamera, output
import cv2
import numpy as np


WIDTH = 1080
HEIGHT = 720

CameraServer.enableLogging()

# Creates UsbCamera and MjpegServer [1] and connects them
camera = UsbCamera("USB Camera 0", 0)
mjpegserver0 = MjpegServer("serve_USB Camera 0", 1181)
mjpegserver0.setSource(camera)

camera.setResolution(WIDTH, HEIGHT)

# Creates the CvSink and connects it to the UsbCamera
cvSink = CvSink("opencv_USB Camera 0");
cvSink.setSource(camera);

# Creates the CvSource and MjpegServer [2] and connects them
output_stream = CvSource("Blur", pixelFormat, 640, 480, 30);
mjpeg_server2 = MjpegServer("serve_Blur", 1182);
mjpeg_server2.setSource(output_stream);

while True:
   time, input_img = CvSink.grabFrame(input_img)

   if time == 0: # There is an error
      output.notifyError(sink.getError())
      continue

   #
   # Insert processing code here
   #

   output.putFrame(processed_img)