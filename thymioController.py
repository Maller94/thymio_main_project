#!/usr/bin/python3
# this imports the camera

from dt_apriltags import Detector
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
import map 
from math import cos, sin, pi, floor
from threading import Thread
from random import random
from adafruit_rplidar import RPLidar

class Thymio:

    ############## INIT ###############################
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
        self.sensorHorizontalValues = []
        self.sensorGroundValues = []
        # initial state is always to map surroundings
        self.state = 'initial'

    def getState(self):
        return self.state
    
    def setState(self,newState):
        self.state = newState


    ############## DRIVER ###############################
    # max speed is 500 = 20cm/s
    def drive(self, left_wheel, right_wheel):      
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])
            
    def stop(self):
        left_wheel = 0
        right_wheel = 0
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])


        def turnRandom(self): 
            r = random.choice([-200, 200])
            l = 0
            if r > 0: 
                l = -200
            else: 
                l = 200

    ############## APRILTAG ###############################
    def apriltagRobotOrientation(self):
        orientation = 'empty'
        while True:
            sleep(2)
            self.camera.resolution = (320, 240)
            self.camera.framerate = 24
            image = np.empty((240, 320, 3), dtype=np.uint8)
            self.camera.capture(image, 'bgr')
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            at_detector = Detector()
            result = at_detector.detect(image)

            if len(result) > 1:
                center_point = 120
                tags = [(abs(x.center[0]-center_point),x.tag_id) for x in result]
                result_tag = min(tags)
                (_,id) = result_tag
                result = id
            else:
                try: 
                    result = result[0].tag_id
                except:
                    result = []
            try:
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
                elif result in [0,17]:
                    orientation = 'L'
                elif result in [1,2]:
                    orientation = 'UL'
                elif result in [19]:
                    orientation = 'found_buddy'
            except:
                orientation = 'empty'
            
            if orientation != 'empty':
                self.apriltagVal = orientation


    ############## SENSORS ###############################
    def sensors_horizontal(self):
        while True: 
            prox_horizontal = self.aseba.GetVariable("thymio-II", "prox.horizontal")
            self.sensorHorizontalValues = prox_horizontal

    def sensors_ground(self):
        while True: 
            prox_ground = self.aseba.GetVariable("thymio-II", "prox.ground.delta")
            self.sensorGroundValues = prox_ground


    ############## LIDAR ###############################
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


    ############## State machines ###############################
    def stateMapping(self): 
        while True:
            if self.getState() == "mapping":
                # This method simululates first state in the state machines (MAPPING)
                s_right = sum(self.scan_data[135:145])/len(self.scan_data[135:145])
                s_left = sum(self.scan_data[215:225])/len(self.scan_data[215:225])
                
                if s_right < 150: 
                    # turn right
                    self.drive(200, 0)
                elif s_left < 150: 
                    # turn left
                    self.drive(0, 200)
                elif self.sensorGroundValues[0] < 500 or self.sensorGroundValues[1] < 500:
                    # drive back, turn right 
                    self.drive(-200,-200)
                    sleep(1)
                    self.turnRandom()
                    sleep(1)
                else: 
                    # drive forward
                    self.drive(200,200)

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
    ############## Controller ###############################
    
    # Mapping
    robot.setState("mapping")
    while True:
        sleep(1)
        try:
            x,y = robot.lidar_orientation_values(robot.apriltagVal)
            print(map.robotPos(x,y))
        except:
            print(robot.apriltagVal)
            pass
        #print(map.robotPos(x,y))
        if(robot.apriltagVal == 'found_buddy'):
            break

    #### change state ####        
    robot.setState('found_buddy')
    robot.stop()
    print(robot.getState())
    ######################
    
    # Search



    # Return
         
        

#------------------- Main loop end ------------------------

if __name__ == '__main__':
    robot = Thymio()
    try:
        ############## Threads ###############################
        sensorGroundThread = Thread(target=robot.sensors_ground)
        sensorGroundThread.daemon = True
        sensorGroundThread.start()
            
        #sensorHorizontalThread = Thread(target=robot.sensors_horizontal)
        #sensorHorizontalThread.daemon = True
        #sensorHorizontalThread.start()

        # Piece of program to prevent RPLIDAREXCEPTION
        try:
            robot.lidar.stop()
            robot.lidar.disconnect()
            robot.PORT_NAME = '/dev/ttyUSB0'
            robot.lidar = RPLidar(None, robot.PORT_NAME)
        except:
            pass
        ###############################################

        scanner_thread = Thread(target=robot.lidar_scan)
        scanner_thread.daemon = True
        scanner_thread.start()
        
        apriltag_thread = Thread(target=robot.apriltagRobotOrientation)
        apriltag_thread.daemon = True
        apriltag_thread.start()

        mapping_thread = Thread(target=robot.stateMapping)
        mapping_thread.daemon = True
        mapping_thread.start()
        
        main()
    except:
        robot.setState("exit")
        sleep(1)
        print("Stopping robot")
        robot.stop()
        exit_now = True
        robot.lidar_stop()
        sleep(1)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")

