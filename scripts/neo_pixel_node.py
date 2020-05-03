#!/usr/bin/env python3

UNKNOWN = "Unknown"
INITIALISED = "Initialised"
BOARDERRROR = "BoardError"
RUNNING = "Running"

import rospy

try:
    import board
    import neopixel
except:
    print(BOARDERRROR)

from visy_neopixel_pkg.msg import *
from std_msgs.msg import String

class NeopixelNode:

    def __init__(self):
        rospy.Subscriber('light_ring_pixels', Neopixels, self.__ctrlLightRingPixelsCallback)
        rospy.Subscriber('status_bar_pixels', Neopixels, self.__ctrlStatusBarPixelsCallback)
        self.__pub = rospy.Publisher('neo_pixel_node_state', String, queue_size=10)
        self.__state = UNKNOWN
        try:
            self.__pixels = neopixel.NeoPixel(board.D21, 20 ,pixel_order=(1,0,2,3))
            self.__init = True
            self.__state = INITIALISED
        except:
            self.__init = False
            self.__state = BOARDERRROR
        for i in range(20):
            self.__pixels[i]=(0,0,0,0)
        self.__pixels.show()

    def __ctrlLightRingPixelsCallback(self,data):
        for i in range(12):
                if self.__init == True :
                        pixel = i + 8
                        self.__pixels[pixel] = (data.pixels[i].r,data.pixels[i].g,data.pixels[i].b,data.pixels[i].w)
                        self.__pixels.show()

    def __ctrlStatusBarPixelsCallback(self,data):
        for i in range(8):
                if self.__init == True :
                        self.__pixels[i] = (data.pixels[i].r,data.pixels[i].g,data.pixels[i].b,data.pixels[i].w)
                        self.__pixels.show()

    def run(self):
        rospy.init_node('neo_pixel_node', anonymous=True)
        if self.__state == INITIALISED : self.__state = RUNNING

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.__pub.publish(self.__state)
            rate.sleep()

if __name__ == '__main__':
    neopixelNode = NeopixelNode()
    neopixelNode.run()
