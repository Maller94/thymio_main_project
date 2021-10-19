#!/usr/bin/python3
# this imports the camera

import pupil_apriltags
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
from threading import Thread
from random import random

class Thymio:
    state = 'initial'

    def __init__(self):
        self.aseba = self.setup()
        self.camera = PiCamera()
        self.camera.start_preview()

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

    def initPicture(self):
        self.camera.resolution = (320, 240)
        self.camera.framerate = 24
        sleep(2)
        image = np.empty((240, 320, 3), dtype=np.uint8)
        self.camera.capture(image, 'bgr')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        at_detector = pupil_apriltags.Detector()
        result = at_detector.detect(image)
        # navigate down the list to retrieve the right information

    def controller(self):
        while True:
            prox_horizontal = self.aseba.GetVariable("thymio-II", "prox.horizontal")
            prox_ground = self.aseba.GetVariable("thymio-II", "prox.ground.delta")

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
    robot = Thymio()

    # do we need this? 
    # thread = Thread(target=robot.controller)
    # thread.daemon = True
    # thread.start()

    # init controller
    #robot.controller()
    while True:
        robot.initPicture()
    
    robot.stop()
#------------------- Main loop end ------------------------

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Stopping robot")
        exit_now = True
        sleep(1)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")
