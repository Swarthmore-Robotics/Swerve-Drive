import json
import time
import sys

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from ntcore import NetworkTableInstance, EventFlags

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfig = None
camera = None

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cameraConfig = CameraConfig()

    # name
    try:
        cameraConfig.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cameraConfig.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cameraConfig.name))
        return False

    # stream properties
    cameraConfig.streamConfig = config.get("stream")

    cameraConfig.config = config

    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # camera
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False

    print(cameras)
    '''[
        {'brightness': 60, 'exposure': 9, 'fps': 30, 'height': 480, 'name': 'Logitech C920', 'path': '/dev/video0', 'pixel format': 'mjpeg', 'properties': [{'name': 'connect_verbose', 'value': 1}, {'name': 'contrast', 'value': 50}, {'name': 'saturation', 'value': 70}, {'name': 'gain', 'value': 0}, {'name': 'power_line_frequency', 'value': 2}, {'name': 'sharpness', 'value': 50}, {'name': 'backlight_compensation', 'value': 0}, {'name': 'exposure_auto_priority', 'value': False}, {'name': 'pan_absolute', 'value': 0}, {'name': 'tilt_absolute', 'value': 0}, {'name': 'focus_absolute', 'value': 0}, {'name': 'focus_auto', 'value': False}, {'name': 'zoom_absolute', 'value': 100}], 'stream': {'properties': []}, 'white balance': 3300, 'width': 640}
        ]
    '''

    readCameraConfig(cameras)

    return True

if __name__ == "__main__":
    configFile = "/boot/frc.json"
    readConfig()

    print("1")

    CameraServer.enableLogging()

    print("Starting camera '{}' on {}".format(cameraConfig.name, cameraConfig.path))

    camera = UsbCamera(cameraConfig.name, cameraConfig.path)
    server = CameraServer.startAutomaticCapture(camera=camera)

    camera.setConfigJson(json.dumps(cameraConfig.config))

    sink = server.getVideo()

    while True:
        time, input_img = sink.grabFrame(input_img)
