#!/usr/bin/env python3
"""Neopixel Node for Vision System."""

import rospy

try:
    import board
    import neopixel
except ImportError:
    rospy.logerr("import failed at neo_pixel_node")

from visy_neopixel_pkg.msg import Neopixels

class NeopixelNode:

    def __init__(self):
        """Class provides ROS Node to support neopixel hardware."""
        self.__numPixels = 0
        self.__init = False
        self.__pixels = ""
        rospy.init_node('neo_pixel_node')
        rospy.Subscriber('neo_pixels', Neopixels, self.__ctrlPixelsCB)

    def __getParams(self):
        try:
            self.__numPixels = rospy.get_param('~number_of_pixels')
            return True
        except Exception:
            rospy.logerr("get params failed at neo_pixel_node")
            return False

    def __setupPixels(self):
        try:
            self.__pixels = neopixel.NeoPixel(board.D21, self.__numPixels ,pixel_order=(1,0,2,3))
            for i in range(self.__numPixels):
                self.__pixels[i]=(0,0,0,0)
            self.__pixels.show()
            return True
        except Exception:
            rospy.logerr("setup failed at neo_pixel_node")
            return False

    def __ctrlPixelsCB(self,data):
        for i in range(data.first,data.last):
            if self.__init == True :
                self.__pixels[i] = (data.pixels[i-data.first].r,data.pixels[i-data.first].g,data.pixels[i-data.first].b,data.pixels[i-data.first].w)
                self.__pixels.show()

    def run(self):
        rate = rospy.Rate(10)
        if self.__getParams() == True:
            if self.__setupPixels() == True:
                self.__init = True
                while not rospy.is_shutdown():
                    rate.sleep()
        else:
            rospy.logerr("failed to start neo_pixel_node")

if __name__ == '__main__':
    neopixelNode = NeopixelNode()
    neopixelNode.run()
