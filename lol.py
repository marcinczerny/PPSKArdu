#!/usr/bin/env python
import rospy
import serial
#import pygame
import os
import time



def talker():
    os.system('mpg321 /home/pi/Jaguar.mp3 &')
    time.sleep(15)
    os.system('mpg321 /home/pi/Fall.mp3 &')


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass