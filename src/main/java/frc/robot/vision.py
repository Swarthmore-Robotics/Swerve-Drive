import json
import time
import sys
import cv2
import numpy as np

from cscore import CameraServer, UsbCamera
import ntcore

configFile = "/boot/frc.json"
WIDTH = 640
HEIGHT = 480
GB = (7, 7)

redlow = (0, 70, 50)
redhigh = (10, 255, 255)

redlow2 = (160, 70, 50)
redhigh2 = (179, 255, 255)

if __name__ == "__main__":
    global configFile, GB
    global redlow, redhigh, redlow2, redhigh2
    print(cv2.__version__)
    with open(configFile) as f:
        config = json.load(f)

    team = config["team"]
    camera_config = config['cameras'][0]

    print("Starting camera '{}' on {}".format(camera_config["name"], camera_config["path"]))

    nt = ntcore.NetworkTableInstance.getDefault()

    # camera.getProperty("name").set(val)
    camera = CameraServer.startAutomaticCapture()
    camera.setResolution(WIDTH, HEIGHT)
    camera.setBrightness(60)
    camera.setWhiteBalanceManual(3300)
    camera.setExposureManual(7)
    camera.getProperty("contrast").set(50)
    camera.getProperty("saturation").set(80)
    camera.getProperty("focus_auto").set(0)

    input_stream = CameraServer.getVideo()
    output_stream = CameraServer.putVideo("Processed", WIDTH, HEIGHT)

    vision_nt = nt.getTable("Vision")

    img = np.zeros((WIDTH, HEIGHT, 3), dtype=np.uint8)

    time.sleep(0.5)

    while True:

        if(input_stream.grabFrame(img) != 0):
            start_time = time.time()

            output_img = np.copy(img)

            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            blur = cv2.GaussianBlur(hsv,gb,0)

            redmask1 = cv2.inRange(blur, redlow, redhigh)
            redmask2 = cv2.inRange(blur, redlow2, redhigh2)

            redmask = cv2.bitwise_or(redmask1, redmask2)

            _, contours, hierarchy = cv2.findContours(redmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # cv2.drawContours(output_img, contours, -1, (0,255,0), 3)

            biggestArea = 0
            biggestX = 0
            biggestY = 0
            biggestH = 0
            biggestW = 0
            for c in contours:
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(image_o2,(x,y),(x+w,y+h),(0,255,0),2)
                a = w*h
                if a > biggestArea:
                    biggestArea = a
                    biggestX = x
                    biggestY = y
                    biggestH = h
                    biggestW = w
            
            # cv2.rectangle(output_img,(biggestX,biggestY),(biggestX+biggestW,biggestY+biggestH),(0,255,0),2)

            end_time = time.time()

            # process_time = end_time - start_time
            # fps = 1/process_time
            # cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))

            output_stream.putFrame(redmask)
            vision_nt.putNumber("Red Area", biggestArea)
            vision_nt.putNumber("Red X", biggestX + (biggestW/2))
            vision_nt.putNumber("Red Y", biggestY + (biggestH/2))

#   JSON format:
#   {
#       "team": <team number>,
#       "ntmode": <"client" or "server", "client" if unspecified>
#       "cameras": [
#           {
#               "name": <camera name>
#               "path": <path, e.g. "/dev/video0">
#               "pixel format": <"MJPEG", "YUYV", etc>   // optional
#               "width": <video mode width>              // optional
#               "height": <video mode height>            // optional
#               "fps": <video mode fps>                  // optional
#               "brightness": <percentage brightness>    // optional
#               "white balance": <"auto", "hold", value> // optional
#               "exposure": <"auto", "hold", value>      // optional
#               "properties": [                          // optional
#                   {
#                       "name": <property name>
#                       "value": <property value>
#                   }
#               ],
#               "stream": {                              // optional
#                   "properties": [
#                       {
#                           "name": <stream property name>
#                           "value": <stream property value>
#                       }
#                   ]
#               }
#           }
#       ]
#       "switched cameras": [
#           {
#               "name": <virtual camera name>
#               "key": <network table key used for selection>
#               // if NT value is a string, it's treated as a name
#               // if NT value is a double, it's treated as an integer index
#           }
#       ]
#   }
