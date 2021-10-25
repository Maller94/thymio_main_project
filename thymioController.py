#!/usr/bin/python3
# this imports the camera

from dt_apriltags import Detector
import queue
from picamera import PiCamera
import os
import numpy as np
import cv2
# initialize asebamedulla in background and wait 0.3s to let
# asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")
import matplotlib.pyplot as plt
from time import sleep
import dbus
import dbus.mainloop.glib
from map import robotPos
from math import cos, sin, pi, floor
from threading import Thread
from random import random
from adafruit_rplidar import RPLidar

class Thymio:
    state = 'initial'

    def __init__(self):
        self.aseba = self.setup()
        self.camera = PiCamera()
        self.camera.start_preview()
        # Setup the RPLidar
        self.PORT_NAME = '/dev/ttyUSB0'
        self.lidar = RPLidar(None, self.PORT_NAME)
        #This is where we store the lidar readings
        self.scan_data = [0]*360
        self.apriltagVal = 'empty'

    def getState(self):
        return self.state
    
    def setState(self,newState):
        self.state = newState

    # max speed is 500 = 20cm/s
    def drive(self, left_wheel_speed, right_wheel_speed):
        print("Left_wheel_speed: " + str(left_wheel_speed))
        print("Right_wheel_speed: " + str(right_wheel_speed))
        
        left_wheel = left_wheel_speed
        right_wheel = right_wheel_speed
        
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def stop(self):
        left_wheel = 0
        right_wheel = 0
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])
        self.camera.stop_preview()

    def apriltagRobotOrientation(self):
        while True:
            sleep(1)
            self.camera.resolution = (320, 240)
            self.camera.framerate = 24
            image = np.empty((240, 320, 3), dtype=np.uint8)
            self.camera.capture(image, 'bgr')
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            at_detector = Detector()
            result = at_detector.detect(image)
            orientation = ''

            try:
                result = result[0].tag_id
                #Orientations
                if result in [3,4,5]:
                    orientation = 'U'
                elif result in [6,7]:
                    orientation = 'UR'
                elif result in [8]:
                    orientation = 'R'
                elif result in [9,10]:
                    orientation = 'DR'
                elif result in [11,12,13]:
                    orientation = 'D'
                elif result in [14,15]:
                    orientation = 'DL'
                elif result in [0,19]:
                    orientation = 'L'
                elif result in [1,2]:
                    orientation = 'UL'
            except:
                result = []
                orientation = ('empty', 'empty')

            self.apriltagVal = (orientation,result)

    def sensors_horizontal(self):
        prox_horizontal = self.aseba.GetVariable("thymio-II", "prox.horizontal")
        return prox_horizontal

    def sensors_ground(self):
        prox_ground = self.aseba.GetVariable("thymio-II", "prox.ground.delta")
        return prox_ground

    def lidar_scan(self):
        for scan in self.lidar.iter_scans():
            #if(self.exit_now):
            #    return
            for (_, angle, distance) in scan:
                self.scan_data[min([359, floor(angle)])] = distance

    def getScanValues(self):
        print(self.scan_data)
        print(len(self.scan_data))
        sleep(2)
    
    def lidar_stop(self):
        self.lidar.stop()
        self.lidar.disconnect()

    # get forward, right, backward, left distances
    def lidar_orientation_values(self, o): 
        try:
            f = self.scan_data[180]
            b = self.scan_data[0]
            r = self.scan_data[270]
            l = self.scan_data[90]
            ur = self.scan_data[225]
            dr = self.scan_data[315]
            ul = self.scan_data[135]
            dl = self.scan_data[45]
            # distances on x,y axes
            x = 0
            y = 0
            
            # set x and y to be distances to the 0,0 coordinate
            if o == "U":
                x = l
                y = b
                return (x,y)
            elif o == "D":
                x = r
                y = f
                return (x,y)
            elif o == "R":
                x = b
                y = r
                return (x,y)
            elif o == "L":
                x = f
                y = l
                return (x,y)
            elif o == "DL": 
                x = ur
                y = ul
                return (x,y)
            elif o == "UL":
                x = ul
                y = l
                return (x,y)
            elif o == "UR":
                x = dl
                y = dr
                return (x,y)
            elif o == "DR":
                x = dr
                y = ur
                return (x,y)
        except:
            pass
    # not using a range of values right now - might be needed
    # need a function to calculate if the distances make sense? 
        # they need to sum either 1920 (horizontal) or 1130(vertical) roughly


############## Bus and aseba setup ######################################

    def setup(self):
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        bus = dbus.SessionBus()
        asebaNetworkObject = bus.get_object("ch.epfl.mobots.Aseba", "/")

        asebaNetwork = dbus.Interface(
            asebaNetworkObject, dbus_interface="ch.epfl.mobots.AsebaNetwork"
        )
        # load the file which is run on the thymio
        asebaNetwork.LoadScripts(
            "thympi.aesl", reply_handler=self.dbusError, error_handler=self.dbusError
        )

        # scanning_thread = Process(target=robot.drive, args=(200,200,))
        return asebaNetwork

    def stopAsebamedulla(self):
        os.system("pkill -n asebamedulla")

    def dbusReply(self):
        # dbus replys can be handled here.
        # Currently ignoring
        pass

    def dbusError(self, e):
        # dbus errors can be handled here.
        # Currently only the error is logged. Maybe interrupt the mainloop here
        print("dbus error: %s" % str(e))


#------------------ Main loop here -------------------------

def main():
    # sensorThread = Thread(target=robot.sensors)
    # sensorThread.daemon = True
    # sensorThread.start()
    
    scanner_thread = Thread(target=robot.lidar_scan)
    scanner_thread.daemon = True
    scanner_thread.start()

    apriltag_thread = Thread(target=robot.apriltagRobotOrientation)
    apriltag_thread.daemon = True
    apriltag_thread.start()

    # Controller        
    while True:
        sleep(1)
        #print(robot.apriltagVal)
        
        #print(robot.lidar_orientation_values(robot.apriltagVal[0]))
        try: 
            x,y = robot.lidar_orientation_values(robot.apriltagVal[0])
            robotPos(x,y)
        except:
            pass
        

#------------------- Main loop end ------------------------

if __name__ == '__main__':
    robot = Thymio()
    q = queue.Queue()
    try:
        main()
    except KeyboardInterrupt:
        print("Stopping robot")
        robot.stop()
        exit_now = True
        robot.lidar_stop()
        sleep(1)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")

