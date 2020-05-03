#!/usr/bin/env python

from visy_neopixel_pkg.srv import *
from visy_neopixel_pkg.msg import *
import rospy

class LightRingNode:

    def __init__(self):
        self.__msg = Neopixels()
        self.__numPixels = 12
        self.__srv = rospy.Service('ctrl_light_ring', LightRing, self.__ctrlLightRing)
        self.__pub = rospy.Publisher('light_ring_pixels', Neopixels, queue_size=10)
        self.__ctrlParam = None

        self.__ctrlFullState=False
        self.__ctrlSpinState=False
        self.__ctrlBlinkState=False
        self.__ctrlFadeState=False

        self.__delayCounter=0
        self.__direction = 0

        self.__blinkState=False

        self.__spinCounter=0

        self.__fadeValue = 0
        self.__fadeSpan = 0

        for i in range(0,self.__numPixels):
            self.__msg.pixels.append(Neopixel())

        return None

    def __resetCtrlStates(self):
        self.__ctrlFullState=False
        self.__ctrlSpinState=False
        self.__ctrlBlinkState=False
        self.__ctrlFadeState=False

        self.__delayCounter=0
        self.__direction = 0

        self.__blinkState=False

        self.__spinCounter=0

        self.__fadeValue = 0
        self.__fadeSpan = 0

    def __ctrlLightRing(self,req):

        state = req.UNKNOWN

        self.__off()

        self.__resetCtrlStates()

        if(req.ctrl >= req.FULL and req.ctrl < req.BLINK_FULL):
            state = self.__ctrlFull(req)

        if(req.ctrl >= req.BLINK_FULL and req.ctrl < req.FADE_FULL):
            state = self.__ctrlBlink(req)

        elif(req.ctrl >= req.FADE_FULL and req.ctrl < req.SPIN_SINGLE_CW):
            state = self.__ctrlFade(req)

        elif(req.ctrl >= req.SPIN_SINGLE_CW and req.ctrl < req.OFF):
            state = self.__ctrlSpin(req)

        return LightRingResponse(state)

    def __ctrlFull(self,req):

        self.__ctrlFullState=True
        self.__ctrlParam = req

        return req.ctrl

    def __ctrlBlink(self,req):

        self.__ctrlBlinkState=True
        self.__ctrlParam = req

        return req.ctrl

    def __ctrlFade(self,req):

        self.__ctrlFadeState=True
        self.__ctrlParam = req
        self.__fadeSpan = self.__calcFadeSpan()

        return req.ctrl

    def __ctrlSpin(self,req):

        self.__ctrlSpinState=True
        self.__ctrlParam = req

        return req.ctrl

    def __full(self):

        for i in range(0,self.__numPixels):
            self.__msg.pixels[i].r = self.__ctrlParam.r
            self.__msg.pixels[i].g = self.__ctrlParam.g
            self.__msg.pixels[i].b = self.__ctrlParam.b
            self.__msg.pixels[i].w = self.__ctrlParam.w
        return

    def __off(self):

        for i in range(0,self.__numPixels):
            self.__msg.pixels[i].r = 0
            self.__msg.pixels[i].g = 0
            self.__msg.pixels[i].b = 0
            self.__msg.pixels[i].w = 0
        return

    def __blink(self):

        for i in range(0,self.__numPixels):

            if(self.__blinkState==True):
                self.__msg.pixels[i].r = self.__ctrlParam.r
                self.__msg.pixels[i].g = self.__ctrlParam.g
                self.__msg.pixels[i].b = self.__ctrlParam.b
                self.__msg.pixels[i].w = self.__ctrlParam.w
            else:
                self.__msg.pixels[i].r = 0
                self.__msg.pixels[i].g = 0
                self.__msg.pixels[i].b = 0
                self.__msg.pixels[i].w = 0

        if(self.__blinkState==False):
            self.__blinkState=True
        else:
            self.__blinkState=False
        return

    def __calcFadeSpan(self):

        span = self.__ctrlParam.r
        if(self.__ctrlParam.g>span):span = self.__ctrlParam.g
        if(self.__ctrlParam.b>span):span = self.__ctrlParam.b
        if(self.__ctrlParam.w>span):span = self.__ctrlParam.w

        return span

    def __fade(self):

        if (self.__fadeValue >= self.__fadeSpan):
            self.__direction = 1
        if (self.__fadeValue <= 0):
            self.__direction = 0

        if(self.__direction == 0):
            self.__fadeValue += 1
        else:
            self.__fadeValue -= 1

        for i in range(0,self.__numPixels):
            if(self.__ctrlParam.r-self.__fadeValue>0):self.__msg.pixels[i].r = self.__ctrlParam.r-self.__fadeValue
            if(self.__ctrlParam.g-self.__fadeValue>0):self.__msg.pixels[i].g = self.__ctrlParam.g-self.__fadeValue
            if(self.__ctrlParam.b-self.__fadeValue>0):self.__msg.pixels[i].b = self.__ctrlParam.b-self.__fadeValue
            if(self.__ctrlParam.w-self.__fadeValue>0):self.__msg.pixels[i].w = self.__ctrlParam.w-self.__fadeValue

        return

    def __spinSingleCw(self):

        for i in range(0,self.__numPixels):

            if(i == self.__spinCounter):
                self.__msg.pixels[i].r = self.__ctrlParam.r
                self.__msg.pixels[i].g = self.__ctrlParam.g
                self.__msg.pixels[i].b = self.__ctrlParam.b
                self.__msg.pixels[i].w = self.__ctrlParam.w
            else:
                self.__msg.pixels[i].r = 0
                self.__msg.pixels[i].g = 0
                self.__msg.pixels[i].b = 0
                self.__msg.pixels[i].w = 0

        if(self.__spinCounter==self.__numPixels-1):
            self.__spinCounter=0
        else:
            self.__spinCounter+=1
        return

    def __spinSingleInvCw(self):

        for i in range(0,self.__numPixels):

            if(i != self.__spinCounter):
                self.__msg.pixels[i].r = self.__ctrlParam.r
                self.__msg.pixels[i].g = self.__ctrlParam.g
                self.__msg.pixels[i].b = self.__ctrlParam.b
                self.__msg.pixels[i].w = self.__ctrlParam.w
            else:
                self.__msg.pixels[i].r = 0
                self.__msg.pixels[i].g = 0
                self.__msg.pixels[i].b = 0
                self.__msg.pixels[i].w = 0

        if(self.__spinCounter==self.__numPixels-1):
            self.__spinCounter=0
        else:
            self.__spinCounter+=1
        return

    def __spinOneFreeCw(self):

        for i in range(0,self.__numPixels):

            if(i % 2 != self.__spinCounter):
                self.__msg.pixels[i].r = self.__ctrlParam.r
                self.__msg.pixels[i].g = self.__ctrlParam.g
                self.__msg.pixels[i].b = self.__ctrlParam.b
                self.__msg.pixels[i].w = self.__ctrlParam.w
            else:
                self.__msg.pixels[i].r = 0
                self.__msg.pixels[i].g = 0
                self.__msg.pixels[i].b = 0
                self.__msg.pixels[i].w = 0

        if(self.__spinCounter==1):
            self.__spinCounter=0
        else:
            self.__spinCounter=1
        return

    def __spinDoubleTop(self):

        for i in range(0,self.__numPixels/2):

            if(i == self.__spinCounter):
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

        if(self.__direction == 0): self.__spinCounter+=1
        else: self.__spinCounter-=1


        if(self.__spinCounter==self.__numPixels/2-1 or self.__spinCounter==0):
            if(self.__direction == 0): self.__direction = 1
            else: self.__direction = 0

        return

    def __step(self):

        #FULL
        if (self.__ctrlFullState==True):
            if(self.__ctrlParam.ctrl == self.__ctrlParam.FULL):
                self.__full()

        #BLINK
        elif (self.__ctrlBlinkState==True):
            if(self.__ctrlParam.ctrl == self.__ctrlParam.BLINK_FULL):
                self.__blink()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.BLINK_FULL_SLOW):
                if(self.__delay(5)):self.__blink()

        #FADE
        elif (self.__ctrlFadeState==True):
            if(self.__ctrlParam.ctrl == self.__ctrlParam.FADE_FULL):
                self.__fade()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.FADE_FULLL_SLOW):
                if(self.__delay(5)):self.__fade()

        #SPIN
        elif (self.__ctrlSpinState==True):
            if(self.__ctrlParam.ctrl == self.__ctrlParam.SPIN_SINGLE_CW):
                self.__spinSingleCw()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.SPIN_SINGLE_CW_SLOW):
                if(self.__delay(5)):self.__spinSingleCw()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.SPIN_SINGLE_INV_CW):
                self.__spinSingleInvCw()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.SPIN_SINGLE_INV_CW_SLOW):
               if(self.__delay(5)): self.__spinSingleInvCw()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.SPIN_ONE_FREE_CW):
                if(self.__delay(5)): state = self.__spinOneFreeCw()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.SPIN_ONE_FREE_CW_FAST):
                self.__spinOneFreeCw()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.SPIN_ONE_FREE_CW_SLOW):
                if(self.__delay(10)): state = self.__spinOneFreeCw()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.SPIN_DOUBLE_TOP):
                self.__spinDoubleTop()

        #OFF
        else:
            self.__off()

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
        rospy.init_node("light_ring_node")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.__step()
            rate.sleep()

if __name__ == "__main__":
    lightRingNode = LightRingNode()
    lightRingNode.run()
