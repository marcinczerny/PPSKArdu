#!/usr/bin/env python
import rospy
import serial
import constants
#import pygame
import os
from std_msgs.msg import String, Int16, Float32 


class state:
    def __init__(self):
        self.limitSwitches = 15
        self.frontDistance = 0
        self.rearDistance = 0
        self.state = constants.CONST_STATE_MOVEMENT
        self.position = 0
        self.floorSensors = 0
    
    def ChangeState(self,newState):
        self.state = newState
    
    def ChangeFrontDistance(self, newDistance):
        self.frontDistance = newDistance
    
    def ChangeRearDistance(self,newDistance):
        self.rearDistance = newDistance
    
    def ChangeLimitSwitches(self,newLimitSwitches):
        self.limitSwitches = newLimitSwitches

    def ChangeFloorSensors(self,newSensors):
        self.floorSensors = newSensors
    
    def GetLimitSwitches(self):
        return self.limitSwitches
    

__ser__ = None
def callback(data):
    global __ser__
    rospy.loginfo(rospy.get_caller_id() + data.data)

def receiveOrders(data):
    global __ser__
    #if data.data =  constants.   

def talker():
    global __ser__
    robotState = state()
    rospy.loginfo("Otwieramy port!")
    __ser__ = serial.Serial('/dev/ttyACM0',115200)
    __ser__.flushInput()
    rospy.loginfo("Otwarlismy port!")
    pub = rospy.Publisher('RaspberryControlWriter',String,queue_size=10)
    rospy.init_node('talker',anonymous=True)
    rate = rospy.Rate(10) #10Hz
    #rospy.Subscriber("RaspberryRead",Int16,)
    pub.publish("JEDZIEMY!")
    rospy.loginfo("Jedziemy!")
    rospy.Subscriber("RaspberryControlReader", String, callback)
    __ser__.write(bytearray([constants.CONST_SERIAL_RPI_INITIALIZED]))
    while not rospy.is_shutdown():
        try:
            while __ser__.in_waiting:
                function = __ser__.read(1)
                if function == constants.CONST_STATE_MOVEMENT:
                    pub.publish("STATE MOVEMENT")
                    rospy.loginfo("MOV")
                    robotState.ChangeState(constants.CONST_STATE_MOVEMENT)
                    #os.system('mpg321 /home/pi/Jaguar.mp3 &')
                elif function == constants.CONST_STATE_OBSTACLE:
                    pub.publish("STATE OBSTACLE")
                    rospy.loginfo("OBST")
                    robotState.ChangeState(constants.CONST_STATE_OBSTACLE)
                    if(robotState.GetLimitSwitches() > 0):
                        pass
                        #os.system('mpg321 /home/pi/Torture.mp3 &')
                elif function == constants.CONST_STATE_STAIRS:
                    pub.publish("STATE STAIRS")
                    rospy.loginfo("STAIRS")
                    robotState.ChangeState(constants.CONST_STATE_STAIRS)
                    #os.system('mpg321 /home/pi/Fall.mp3 &')
                elif function == constants.CONST_STATE_STOP:
                    pub.publish("STATE STOP")
                    rospy.loginfo("STOP")
                    robotState.ChangeState(constants.CONST_STATE_STOP)
                elif function == constants.CONST_FLOOR:
                    while not __ser__.in_waiting:
                        pass
                    floorSensors = __ser__.read(1)
                    robotState.ChangeFloorSensors(floorSensors)
                elif function == constants.CONST_OBSTACLE_SONAR_BACK:
                    while not __ser__.in_waiting:
                        pass
                    backSonar = __ser__.read(1)
                    robotState.ChangeRearDistance(backSonar)
                elif function == constants.CONST_OBSTACLE_SONAR_FRONT:
                    while not __ser__.in_waiting:
                        pass
                    frontSonar = __ser__.read(1)
                    robotState.ChangeFrontDistance(frontSonar)
                elif function == constants.CONST_OBSTACLE_SWITCHES:
                    while not __ser__.in_waiting:
                        pass
                    limitSwitches = __ser__.read(1)
                    robotState.ChangeLimitSwitches(limitSwitches)
            rate.sleep()
        except rospy.ROSInterruptException:
            pass        


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass