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
GB = (11, 11)

redlow = (0, 70, 110)
redhigh = (8, 255, 255)

redlow2 = (160, 70, 110)
redhigh2 = (179, 255, 255)

if __name__ == "__main__":
    print(cv2.__version__)
    with open(configFile) as f:
        config = json.load(f)

    team = config["team"]
    camera_config = config['cameras'][0]

    print("Starting camera '{}' on {}".format(camera_config["name"], camera_config["path"]))

    nt = ntcore.NetworkTableInstance.getDefault()
    # nt.setSeverTeam(6593)
    nt.startClient4("rasp pi client")
    nt.setServerTeam(6593)
    nt.startDSClient()
    # nt.setServer("host", ntcore.NetworkTableInstance.kDefaultPort4)

    # camera.getProperty("name").set(val)
    camera = CameraServer.startAutomaticCapture()
    camera.setResolution(WIDTH, HEIGHT)
    camera.setBrightness(60)
    camera.setWhiteBalanceManual(3300)
    camera.setExposureManual(2)
    camera.getProperty("contrast").set(50)
    camera.getProperty("saturation").set(80)
    camera.getProperty("focus_auto").set(0)
    camera.getProperty("gain").set(50)

    input_stream = CameraServer.getVideo()
    output_stream = CameraServer.putVideo("Processed RaspPi", WIDTH, HEIGHT)

    vision_nt = nt.getTable("Vision")
    biggestArea = vision_nt.getDoubleTopic("redA").publish()
    biggestX = vision_nt.getDoubleTopic("redX").publish()
    biggestY = vision_nt.getDoubleTopic("redY").publish()

    img = np.zeros((WIDTH, HEIGHT, 3), dtype=np.uint8)

    counter = 0

    time.sleep(0.5)

    while True:

        result, img = input_stream.grabFrame(img)

        if(result != 0):

            start_time = time.time()

            # print('image shape, min, max', img.shape, img.min(axis=(0,1)), img.max(axis=(0,1)))

            output_img = np.copy(img)

            blur = cv2.GaussianBlur(img, GB, 0)

            hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
            # hsv = cv2.cvtColor(blur, cv2.COLOR_RGB2HSV)


            redmask1 = cv2.inRange(hsv, redlow, redhigh)
            # redmask = cv2.inRange(hsv, redlow, redhigh)

            # print('redlow', redlow)
            # print('redhi', redhigh)
            # print('mask', redmask.min(), redmask.max(), redmask.sum())

            redmask2 = cv2.inRange(hsv, redlow2, redhigh2)

            redmask = cv2.bitwise_or(redmask1, redmask2)

            contours, hierarchy = cv2.findContours(redmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # cv2.drawContours(output_img, contours, -1, (0,255,0), 3)

            biggestAreaTemp = 0.0
            biggestXTemp = 0.0
            biggestYTemp = 0.0
            biggestHTemp = 0.0
            biggestWTemp = 0.0
            tl = ()
            br = ()
            cent_coord = ()
            # print(len(contours))
            for c in contours:
                x,y,w,h = cv2.boundingRect(c)
                # cv2.rectangle(output_img,(x,y),(x+w,y+h),(0,255,0),2)
                a = w*h
                centerx = x + (w/2)
                centery = y + (h/2)
                if a > biggestAreaTemp:
                    biggestAreaTemp = a
                    biggestXTemp = centerx
                    biggestYTemp = centery
                    biggestHTemp = h
                    biggestWTemp = w
                    tl = (x, y)
                    br = (x + w, y + h)
                    cent_coord = (int(x + w/2), int(y + h/2))
            
            if (len(tl) != 0 and  len(br) != 0 and len(cent_coord) != 0):
                cv2.rectangle(output_img, tl, br,(0,255,0),2)
                cv2.circle(output_img, cent_coord, 5, (255,0,0), -1)
            # print(cent_coord)
            # print(br)

            end_time = time.time()

            # process_time = end_time - start_time
            # fps = 1/process_time
            # cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))

            # counter = counter + 1
            # biggestXTemp = counter

            print(biggestAreaTemp)
            print(biggestXTemp)
            
            output_stream.putFrame(output_img)
            biggestArea.set(biggestAreaTemp)
            biggestX.set(biggestXTemp)
            biggestY.set(biggestYTemp)

            print(end_time - start_time)
            

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