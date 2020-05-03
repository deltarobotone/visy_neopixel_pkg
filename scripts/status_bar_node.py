#!/usr/bin/env python

from visy_neopixel_pkg.srv import *
from visy_neopixel_pkg.msg import *
import rospy

class StatusBarNode:

    def __init__(self):
        self.__msg = Neopixels()
        self.__numPixels = 8
        self.__srv = rospy.Service('ctrl_status_bar', StatusBar, self.__ctrlStatusBar)
        self.__pub = rospy.Publisher('status_bar_pixels', Neopixels, queue_size=10)
        self.__flowState=False
        self.__flowCounter=0
        self.__delayCounter=0
        self.__direction = 0
        self.__ctrlParam = None

        for i in range(0,self.__numPixels):
            self.__msg.pixels.append(Neopixel())

        return None

    def __resetCtrlStates(self):
        self.__flowState=False
        self.__flowCounter=0
        self.__delayCounter=0
        self.__direction = 0

    def __ctrlStatusBar(self,req):

        state = req.UNKNOWN

        self.__resetCtrlStates()

        if(req.ctrl == req.FULL):
            state = self.__ctrlFull(req)

        elif(req.ctrl >= 20 and req.ctrl < 60):
            state = self.__ctrlFlow(req)

        return StatusBarResponse(state)

    def __ctrlFull(self,req):

        for i in range(0,self.__numPixels):
            self.__msg.pixels[i].r = req.r
            self.__msg.pixels[i].g = req.g
            self.__msg.pixels[i].b = req.b
            self.__msg.pixels[i].w = req.w

        return req.ctrl

    def __ctrlFlow(self,req):

        self.__flowState=True
        self.__ctrlParam = req

        return req.ctrl

    def __flowSingleCw(self):

        for i in range(0,self.__numPixels):

            if(i == self.__flowCounter):
                self.__msg.pixels[i].r = self.__ctrlParam.r
                self.__msg.pixels[i].g = self.__ctrlParam.g
                self.__msg.pixels[i].b = self.__ctrlParam.b
                self.__msg.pixels[i].w = self.__ctrlParam.w
            else:
                self.__msg.pixels[i].r = 0
                self.__msg.pixels[i].g = 0
                self.__msg.pixels[i].b = 0
                self.__msg.pixels[i].w = 0

        if(self.__flowCounter==self.__numPixels-1):
            self.__flowCounter=0
        else:
            self.__flowCounter+=1
        return

    def __flowSingleInvCw(self):

        for i in range(0,self.__numPixels):

            if(i != self.__flowCounter):
                self.__msg.pixels[i].r = self.__ctrlParam.r
                self.__msg.pixels[i].g = self.__ctrlParam.g
                self.__msg.pixels[i].b = self.__ctrlParam.b
                self.__msg.pixels[i].w = self.__ctrlParam.w
            else:
                self.__msg.pixels[i].r = 0
                self.__msg.pixels[i].g = 0
                self.__msg.pixels[i].b = 0
                self.__msg.pixels[i].w = 0

        if(self.__flowCounter==self.__numPixels-1):
            self.__flowCounter=0
        else:
            self.__flowCounter+=1
        return

    def __flowOneFreeCw(self):

        for i in range(0,self.__numPixels):

            if(i % 2 != self.__flowCounter):
                self.__msg.pixels[i].r = self.__ctrlParam.r
                self.__msg.pixels[i].g = self.__ctrlParam.g
                self.__msg.pixels[i].b = self.__ctrlParam.b
                self.__msg.pixels[i].w = self.__ctrlParam.w
            else:
                self.__msg.pixels[i].r = 0
                self.__msg.pixels[i].g = 0
                self.__msg.pixels[i].b = 0
                self.__msg.pixels[i].w = 0

        if(self.__flowCounter==1):
            self.__flowCounter=0
        else:
            self.__flowCounter=1
        return

    def __flowDoubleTop(self):

        for i in range(0,self.__numPixels/2):

            if(i == self.__flowCounter):
                self.__msg.pixels[i].r = self.__ctrlParam.r
                self.__msg.pixels[i].g = self.__ctrlParam.g
                self.__msg.pixels[i].b = self.__ctrlParam.b
                self.__msg.pixels[i].w = self.__ctrlParam.w

                self.__msg.pixels[self.__numPixels-i-1].r = self.__ctrlParam.r
                self.__msg.pixels[self.__numPixels-i-1].g = self.__ctrlParam.g
                self.__msg.pixels[self.__numPixels-i-1].b = self.__ctrlParam.b
                self.__msg.pixels[self.__numPixels-i-1].w = self.__ctrlParam.w
            else:
                self.__msg.pixels[i].r = 0
                self.__msg.pixels[i].g = 0
                self.__msg.pixels[i].b = 0
                self.__msg.pixels[i].w = 0

                self.__msg.pixels[self.__numPixels-i-1].r = 0
                self.__msg.pixels[self.__numPixels-i-1].g = 0
                self.__msg.pixels[self.__numPixels-i-1].b = 0
                self.__msg.pixels[self.__numPixels-i-1].w = 0

        if(self.__direction == 0): self.__flowCounter+=1
        else: self.__flowCounter-=1


        if(self.__flowCounter==self.__numPixels/2-1 or self.__flowCounter==0):
            if(self.__direction == 0): self.__direction = 1
            else: self.__direction = 0

        return

    def __step(self):
        if (self.__flowState==True):
            if(self.__ctrlParam.ctrl == self.__ctrlParam.FLOW_SINGLE_CW):
                self.__flowSingleCw()

            if(self.__ctrlParam.ctrl == self.__ctrlParam.FLOW_SINGLE_CW_SLOW):
                if(self.__delay(5)):self.__flowSingleCw()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.FLOW_SINGLE_INV_CW):
                self.__flowSingleInvCw()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.FLOW_SINGLE_INV_CW_SLOW):
               if(self.__delay(5)): self.__flowSingleInvCw()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.FLOW_ONE_FREE_CW):
                if(self.__delay(5)): state = self.__flowOneFreeCw()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.FLOW_ONE_FREE_CW_FAST):
                state = self.__flowOneFreeCw()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.FLOW_ONE_FREE_CW_SLOW):
                if(self.__delay(10)): state = self.__flowOneFreeCw()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.FLOW_DOUBLE_TOP):
                state = self.__flowDoubleTop()

        self.__pub.publish(self.__msg)
        return

    def __delay(self, delay):
        if(self.__delayCounter==delay):
            self.__delayCounter=0
            return True
        else:
            self.__delayCounter+=1
            return False

    def run(self):
        rospy.init_node("status_bar_node")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.__step()
            rate.sleep()

if __name__ == "__main__":
    statusBarNode = StatusBarNode()
    statusBarNode.run()
