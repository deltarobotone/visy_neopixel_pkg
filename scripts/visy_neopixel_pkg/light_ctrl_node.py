#!/usr/bin/env python
"""Light Control Node for Vision System."""

from visy_neopixel_pkg.srv import LightCtrl,LightCtrlResponse,PixelCtrl,PixelCtrlResponse
from visy_neopixel_pkg.msg import Neopixels,Neopixel
import rospy

class LightCtrlNode:

    def __init__(self):
        """Class provides ROS Node for light control functions."""
        self.__msg = Neopixels()
        self.__firstPixel = 0
        self.__lastPixel = 0
        self.__numPixel = 0

        self.__ctrlParam = None

        self.__ctrlFullState=False
        self.__ctrlSpinState=False
        self.__ctrlBlinkState=False
        self.__ctrlFadeState=False
        self.__ctrlPixel=False

        self.__delayCounter=0
        self.__direction = 0

        self.__blinkState=False

        self.__spinCounter=0

        self.__fadeValue = 0
        self.__fadeSpan = 0

        rospy.init_node("~")
        self.__srv = rospy.Service('light_ctrl', LightCtrl, self.__lightCtrlCB)
        self.__srv = rospy.Service('pixel_ctrl', PixelCtrl, self.__pixelCtrlCB)
        self.__pub = rospy.Publisher('~/neo_pixels', Neopixels, queue_size=1)

        return None

    def __getParams(self):
        try:
            self.__firstPixel = rospy.get_param('~first_pixel')
            self.__lastPixel = rospy.get_param('~last_pixel')
            if self.__firstPixel > 0: self.__firstPixel = self.__firstPixel - 1
            self.__numPixel = self.__lastPixel-self.__firstPixel
            for i in range(self.__numPixel):
                self.__msg.pixels.append(Neopixel())
                rospy.loginfo("init pixel: " + str(i+1))
            return True
        except Exception:
            rospy.logerr("get params failed at light_ctrl_node")
            return False

    def __resetCtrlStates(self):
        self.__ctrlFullState=False
        self.__ctrlSpinState=False
        self.__ctrlBlinkState=False
        self.__ctrlFadeState=False
        self.__ctrlPixel=False
        self.__delayCounter=0
        self.__direction = 0
        self.__blinkState=False
        self.__spinCounter=0
        self.__fadeValue = 0
        self.__fadeSpan = 0

    def __pixelCtrlCB(self,req):
        self.__resetCtrlStates()
        self.__off()

        if(req.pos > 0 and req.pos <= self.__lastPixel-self.__firstPixel):
            self.__msg.pixels[req.pos-1] = req.pixel
            self.__ctrlPixel=True
            return PixelCtrlResponse(True)
        else:
            return PixelCtrlResponse(False)


    def __lightCtrlCB(self,req):

        self.__off()

        state = req.OFF

        self.__resetCtrlStates()

        if(req.ctrl >= req.FULL and req.ctrl < req.BLINK_FULL):
            state = self.__ctrlFull(req)

        if(req.ctrl >= req.BLINK_FULL and req.ctrl < req.FADE_FULL):
            state = self.__ctrlBlink(req)

        elif(req.ctrl >= req.FADE_FULL and req.ctrl < req.SPIN_SINGLE_CW):
            state = self.__ctrlFade(req)

        elif(req.ctrl >= req.SPIN_SINGLE_CW and req.ctrl < req.OFF):
            state = self.__ctrlSpin(req)

        return LightCtrlResponse(state)

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
        for i in range(0,self.__numPixel):
            self.__msg.pixels[i] = self.__ctrlParam.pixel
        return

    def __off(self):
        for i in range(0,self.__numPixel):
            self.__msg.pixels[i] = Neopixel()
        return

    def __blink(self):

        for i in range(0,self.__numPixel):

            if(self.__blinkState==True):
                self.__msg.pixels[i] = self.__ctrlParam.pixel
            else:
                self.__msg.pixels[i] = Neopixel()

        if(self.__blinkState==False):
            self.__blinkState=True
        else:
            self.__blinkState=False
        return

    def __calcFadeSpan(self):

        span = 0
        if(self.__ctrlParam.pixel.r>span):span = self.__ctrlParam.pixel.r
        if(self.__ctrlParam.pixel.g>span):span = self.__ctrlParam.pixel.g
        if(self.__ctrlParam.pixel.b>span):span = self.__ctrlParam.pixel.b
        if(self.__ctrlParam.pixel.w>span):span = self.__ctrlParam.pixel.w

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

        for i in range(0,self.__numPixel):
            if(self.__ctrlParam.pixel.r-self.__fadeValue>0):self.__msg.pixels[i].r = self.__ctrlParam.pixel.r-self.__fadeValue
            if(self.__ctrlParam.pixel.g-self.__fadeValue>0):self.__msg.pixels[i].g = self.__ctrlParam.pixel.g-self.__fadeValue
            if(self.__ctrlParam.pixel.b-self.__fadeValue>0):self.__msg.pixels[i].b = self.__ctrlParam.pixel.b-self.__fadeValue
            if(self.__ctrlParam.pixel.w-self.__fadeValue>0):self.__msg.pixels[i].w = self.__ctrlParam.pixel.w-self.__fadeValue

        return

    def __spinSingleCw(self):

        for i in range(0,self.__numPixel):

            if(i == self.__spinCounter):
                self.__msg.pixels[i] = self.__ctrlParam.pixel
            else:
                self.__msg.pixels[i].r = Neopixel()

        if(self.__spinCounter==self.__numPixel-1):
            self.__spinCounter=0
        else:
            self.__spinCounter+=1
        return

    def __spinSingleInvCw(self):

        for i in range(0,self.__numPixel):

            if(i != self.__spinCounter):
                self.__msg.pixels[i] = self.__ctrlParam.pixel
            else:
                self.__msg.pixels[i].r = Neopixel()

        if(self.__spinCounter==self.__numPixel-1):
            self.__spinCounter=0
        else:
            self.__spinCounter+=1
        return

    def __spinOneFreeCw(self):

        for i in range(0,self.__numPixel):

            if(i % 2 != self.__spinCounter):
                self.__msg.pixels[i] = self.__ctrlParam.pixel
            else:
                self.__msg.pixels[i].r = Neopixel()

        if(self.__spinCounter==1):
            self.__spinCounter=0
        else:
            self.__spinCounter=1
        return

    def __spinDoubleTop(self):

        for i in range(0,self.__numPixel/2):

            if(i == self.__spinCounter):
                self.__msg.pixels[i] = self.__ctrlParam.pixel
                self.__msg.pixels[self.__numPixel-i-1] = self.__ctrlParam.pixel

            else:
                self.__msg.pixels[i] = Neopixel()
                self.__msg.pixels[self.__numPixel-i-1] = Neopixel()

        if(self.__direction == 0): self.__spinCounter+=1
        else: self.__spinCounter-=1


        if(self.__spinCounter==self.__numPixel/2-1 or self.__spinCounter==0):
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
                if(self.__delay(5)): self.__spinOneFreeCw()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.SPIN_ONE_FREE_CW_FAST):
                self.__spinOneFreeCw()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.SPIN_ONE_FREE_CW_SLOW):
                if(self.__delay(10)): self.__spinOneFreeCw()

            elif(self.__ctrlParam.ctrl == self.__ctrlParam.SPIN_DOUBLE_TOP):
                self.__spinDoubleTop()
        #OFF
        else:
            if (self.__ctrlPixel==False): 
                self.__off()

        self.__msg.first = self.__firstPixel
        self.__msg.last = self.__lastPixel
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
        rate = rospy.Rate(10)
        if self.__getParams() == True:
            while not rospy.is_shutdown():
                self.__step()
                rate.sleep()
        else:
            rospy.logerr("failed to start light_ctrl_node")

if __name__ == "__main__":
    lightCtrlNode = LightCtrlNode()
    lightCtrlNode.run()
