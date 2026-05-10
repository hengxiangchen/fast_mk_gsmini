import os
import cv2
import yaml
from utils.gs_device import FastCamera

"""
This script demonstrates how to use the FastCamera class from the gs_sdk package.

It loads a configuration file, initializes the FastCamera, and streaming images with low latency.
This script is only for GelSight Mini so far as only GelSight Mini has the frame dropping issue.

Usage:
    python gsmini_driver.py 

Press any key to quit the streaming session.
"""

config_dir = os.path.join(os.path.dirname(__file__), "configs")
import time
import rospy

def config_reader(config_path):
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
        device_name = config["device_name"]
        imgh = config["imgh"]
        imgw = config["imgw"]
        raw_imgh = config["raw_imgh"]
        raw_imgw = config["raw_imgw"]
        framerate = config["framerate"]
    return device_name, imgh, imgw, raw_imgh, raw_imgw, framerate

class Gsmini:
    def __init__(self):
        # Load the device configuration
        self.config_path = os.path.join(config_dir, "gsmini.yaml")
        self.device_name, self.imgh, self.imgw, self.raw_imgh, self.raw_imgw, self.framerate = config_reader(self.config_path)

        self.dev_type = self.device_name
        self._connect_image()
        # rospy.init_node("gsmini", anonymous=True)
        
    def connect(self):
        # print(self.device)
        assert self.device is not None, "Warning: unable to open video source %d" % (self.dev_id)
        return True if self.device is not None else False
        # print("Connect to %s at video source %d" % (self.dev_type, self.dev_id)) 

    def _connect_image(self):
        # Create device and stream the device
        self.device = FastCamera(self.device_name, self.imgh, self.imgw, self.raw_imgh, self.raw_imgw, self.framerate)
        self.device.connect()
        self.dev_id = self.device.dev_id

    def _get_image(self):
        return self.device.get_image(), rospy.Time.now()

    def fast_stream_device(self):
        # self._connect_image()
        print("\nPrss any key to quit.\n")
        t0 = time.perf_counter()
        count = 0
        fps = 0
        while True:
            image = self.device.get_image()
            t1 = time.perf_counter()
            count += 1
            if t1 - t0 > 1.0:
                fps = count / (t1 - t0)
                print(fps)
                count = 0
                fps = 0
                t0 = t1
            cv2.imshow(self.device_name, image)
            key = cv2.waitKey(1)
            if key != -1:
                break

    def release(self):
        self.device.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    gsmini = Gsmini()
    # while True:
    #     image, _ = gsmini._get_image()
    #     cv2.imshow("gsmini", image)
    #     key = cv2.waitKey(1)
    #     if key != -1:
    #         break
    # gsmini.connect()
    gsmini.fast_stream_device()
    gsmini.release()
