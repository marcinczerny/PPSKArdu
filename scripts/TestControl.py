#!/usr/bin/env python
import rospy
import serial
import constants
import sys
from select import select
#import pygame
import os
import time
from std_msgs.msg import String, Int16, Float32, Int16MultiArray
#from ppsk.msg import Control
   
class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
            print(ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch





def talker():
    
    pub = rospy.Publisher('RaspberryControlReader',Int16,queue_size=10)
    rospy.init_node('tester',anonymous=True)
    rate = rospy.Rate(1) #20Hz
    rospy.loginfo("StartTester!")
    x = Int16()
    x.data = constants.CONST_STOP * 1000
    
    time.sleep(10)
    rospy.loginfo(x.data)
    pub.publish(x)

    time.sleep(10)

    x.data = constants.CONST_START * 1000
    rospy.loginfo(x.data)
    pub.publish(x)
    #k = _GetchUnix()
    #rospy.loginfo(k)
    # bufferSerial = b""
    while not rospy.is_shutdown():
        try:
            pass
            # timeout = 1
            # rlist, _, _ = select([sys.stdin], [], [], timeout)
            # if rlist:
            #     s = sys.stdin.readline()
            #     if(s == "w")
            #     print s
            # else:
            #     print "No input. Moving on..."
            # getch = _Getch()

            # rospy.loginfo(getch)
        except rospy.ROSInterruptException:
            break                
                 


if __name__ == '__main__':
    #try:
        talker()
    #except rospy.ROSInterruptException:
    #    pass