#!/usr/bin/python3
# this imports the camera

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
    def __init__(self):
        self.aseba = self.setup()

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

    def testCamera(self):
        camera = PiCamera()
        print("Camera test")
        camera.start_preview()
        sleep(5)
        #we capture to openCV compatible format
        #you might want to increase resolution
        camera.resolution = (320, 240)
        camera.framerate = 24
        sleep(2)
        image = np.empty((240, 320, 3), dtype=np.uint8)
        camera.capture(image, 'bgr')
        cv2.imwrite('out.png', image)
        camera.stop_preview()
        print("saved image to out.png")

    def sens(self):
        # Timer used for the testing phase
        # for cnt in range(5000):
        sensors = self.aseba.GetVariable("thymio-II", "prox.horizontal")
        # print(f'timer {i}')
            #print("Sensing:")
            
            # Controller
        #     if sensors[0] > 4500 or sensors[1] > 4500 or sensors[2] > 4500 or sensors[3] > 4500 or sensors[4] > 4500:
        #         self.drive(-100,100)
        #     else:                
        #         self.drive(-100,100)
        # self.stop()
        return sensors

############## Bus and aseba setup ######################################

    def setup(self):
        print("Setting up")
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

    # Make a robot controller (Talk with martin)

    robot.testCamera()

    # robot.sens() 
    # thread = Thread(target=robot.sens)
    # thread.daemon = True
    # thread.start()

    #robot.drive(200, 200)
    #sleep(15)
    #robot.stop()
       

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
