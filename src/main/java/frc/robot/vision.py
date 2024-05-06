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

redlow2 = (160, 190, 120)
redhigh2 = (179, 255, 255)

yellowlow = (22, 70, 110)
yellowhigh = (35, 255, 255)

biggestAreaRed = 0.0
biggestXRed = 0.0
biggestYRed = 0.0

biggestAreaYellow = 0.0
biggestXYellow = 0.0
biggestYYellow = 0.0

DEBUG = True
THRESH = 15

def getBiggestColor(mask, output_img):
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(output_img, contours, -1, (0,255,0), 3)

    biggestArea = 0.0
    biggestX = 0.0
    biggestY = 0.0
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
        if a > biggestArea:
            biggestArea = a
            biggestX = centerx
            biggestY = centery
            biggestHTemp = h
            biggestWTemp = w
            tl = (x, y)
            br = (x + w, y + h)
            cent_coord = (int(x + w/2), int(y + h/2))
    
    if (len(tl) != 0 and  len(br) != 0 and len(cent_coord) != 0):
        cv2.rectangle(output_img, tl, br,(0,0,255),2)
        cv2.circle(output_img, cent_coord, 5, (255,0,0), -1)
    # print(cent_coord)
    # print(br)

    cv2.line(output_img, (WIDTH//2, 0), (WIDTH//2, HEIGHT), (255, 255, 255), 2)
    cv2.line(output_img, (WIDTH//2 + THRESH, 0), (WIDTH//2 + THRESH, HEIGHT), (255, 255, 255), 1)
    cv2.line(output_img, (WIDTH//2 - THRESH, 0), (WIDTH//2 - THRESH, HEIGHT), (255, 255, 255), 1)

    return biggestArea, biggestX, biggestY, output_img

def debugVision(input_stream, output_stream, img):

    blur = np.zeros((WIDTH, HEIGHT, 3), dtype=np.uint8)
    hsv = np.zeros((WIDTH, HEIGHT, 3), dtype=np.uint8)
    redmask1 = np.zeros((WIDTH, HEIGHT, 1), dtype=np.uint8)
    redmask2 = np.zeros((WIDTH, HEIGHT, 1), dtype=np.uint8)
    redmask = np.zeros((WIDTH, HEIGHT, 1), dtype=np.uint8)
    yellowmask = np.zeros((WIDTH, HEIGHT, 1), dtype=np.uint8)

    biggestAreaRedTemp = 0.0
    biggestXRedTemp = 0.0
    biggestYRedTemp = 0.0

    biggestAreaYellowTemp = 0.0
    biggestXYellowTemp = 0.0
    biggestYYellowTemp = 0.0

    while True:

        # grab image from camera
        result, img = input_stream.grabFrame(img)
        output_img = np.copy(img)

        if(result != 0):

            start_time = time.time()

            # print('image shape, min, max', img.shape, img.min(axis=(0,1)), img.max(axis=(0,1)))

            blur = cv2.GaussianBlur(img, GB, 0) # apply gaussian blur

            hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV) # convert to HSV colorspace

            # red threshold
            redmask1 = cv2.inRange(hsv, redlow, redhigh)
            redmask2 = cv2.inRange(hsv, redlow2, redhigh2)
            redmask = cv2.bitwise_or(redmask1, redmask2)
            # print('redlow', redlow)
            # print('redhi', redhigh)
            # print('mask', redmask.min(), redmask.max(), redmask.sum())

            # yellow threshold
            yellowmask = cv2.inRange(hsv, yellowlow, yellowhigh)

            biggestAreaRedTemp, biggestXRedTemp, biggestYRedTemp, output_img = getBiggestColor(redmask, output_img)
            biggestAreaYellowTemp, biggestXYellowTemp, biggestYYellowTemp, output_img = getBiggestColor(yellowmask, output_img)


            
            output_stream.putFrame(output_img)
            biggestAreaRed.set(biggestAreaRedTemp)
            biggestXRed.set(biggestXRedTemp)
            biggestYRed.set(biggestYRedTemp)

            biggestAreaYellow.set(biggestAreaYellowTemp)
            biggestXYellow.set(biggestXYellowTemp)
            biggestYYellow.set(biggestYYellowTemp)

            end_time = time.time()

            print(end_time - start_time)


if __name__ == "__main__":
    global biggestAreaRed, biggestAreaYellow, biggestXRed, biggestXYellow, biggestYRed, biggestYYellow
    # print(cv2.__version__)
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
    biggestAreaRed = vision_nt.getDoubleTopic("redA").publish()
    biggestXRed = vision_nt.getDoubleTopic("redX").publish()
    biggestYRed = vision_nt.getDoubleTopic("redY").publish()

    biggestAreaYellow = vision_nt.getDoubleTopic("yellowA").publish()
    biggestXYellow = vision_nt.getDoubleTopic("yellowX").publish()
    biggestYYellow = vision_nt.getDoubleTopic("yellowY").publish()

    img = np.zeros((WIDTH, HEIGHT, 3), dtype=np.uint8)

    time.sleep(0.5)

    debugVision(input_stream, output_stream, img)