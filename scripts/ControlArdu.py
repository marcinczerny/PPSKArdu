#!/usr/bin/env python
import rospy
import serial
import constants
import datetime as dt
#import pygame
import os
import time
from std_msgs.msg import String, Int16, Float32, Int16MultiArray
#from ppsk.msg import Control
class order:
    def __init__(self):
        self.order = []
        self.OrderProperties=[]
        self.SerialMessage = b""
        self.Change = 0
        self.Response = ""
        self.previousControlOrder = dt.datetime(1900,1,1,12,00,00,00,None)
    
    def ChangeOrder(self,newOrder):
        self.order.append(newOrder)

    def ChangeOrderProperties(self,newProperties):
        self.OrderProperties = newProperties

    
    def ChangeOrderAndProperties(self,newOrder,newProperties):
        self.OrderProperties.append(newProperties)
        self.order.append(newOrder)

    def ReadOrder(self):
        return (self.order[0],self.OrderProperties[0])

    def CheckChange(self):
        #return self.Change 
        return len(self.order)

    def ReadSerialMessage(self):
        return self.SerialMessage

    def DecideIfItIsARequest(self):
        requestList = [constants.CONST_READ_STATE,constants.CONST_READ_DISTANCE_FRONT,constants.CONST_READ_DISTANCE_REAR,constants.CONST_DIRECTION,constants.CONST_READ_FLOOR,constants.CONST_READ_LIMIT_SWITCHES]
        if self.order[0] in requestList:
            return True


    def GenerateResponse(self,robotState,order = None):

        if order is None:
            temp = self.order.pop()
            self.OrderProperties.pop()
        else:
            temp = order
        if temp == constants.CONST_READ_STATE:
            self.Response = constants.CONST_RESPONSE_STATE + str(robotState.state) + constants.CONST_RESPONSE_POSITION + str(robotState.position)
            return True
        if temp == constants.CONST_READ_LIMIT_SWITCHES:
            self.Response = constants.CONST_RESPONSE_LIMIT_SWITCHES + str(robotState.limitSwitches)
            return True
        if temp == constants.CONST_READ_DISTANCE_FRONT:
            self.Response = constants.CONST_RESPONSE_DISTANCE_FRONT + str(robotState.frontDistance)
            return True
        if temp == constants.CONST_READ_DISTANCE_REAR:
            self.Response = constants.CONST_RESPONSE_DISTANCE_READ + str(robotState.rearDistance)
            return True
        if temp == constants.CONST_READ_FLOOR:
            self.Response = constants.CONST_RESPONSE_FLOOR + str(robotState.floorSensors)
            return True
        else:
            return False

    def GenerateSerialMessage(self,robotSt):
        robotSt = state()
        
        temp = self.order.pop()
        tempProperties = self.OrderProperties.pop() 
        if temp == constants.CONST_START and robotSt.state == constants.CONST_STOP:
            
            rospy.loginfo("Generuje start")
            
            self.SerialMessage = (bytearray([constants.CONST_SERIAL_RPI_START])) 
            return True
        elif temp == constants.CONST_DIRECTION and robotSt.state == constants.CONST_STATE_MOVEMENT :
            if robotSt.orderPosition != tempProperties:
                nextPosition = int(robotSt.position + (tempProperties - 128)*0.3)
                robotSt.position = nextPosition
                self.SerialMessage = (bytearray([constants.CONST_SERIAL_RPI_DIRECTION, nextPosition]))
                robotSt.orderPosition = tempProperties
                return True
            else:
                return False
        elif temp == constants.CONST_SPEED and robotSt.state != constants.CONST_STATE_STOP:
            if robotSt.orderSpeed != tempProperties:
                if robotSt.state == constants.CONST_STATE_OBSTACLE or robotSt.state == constants.CONST_STATE_STAIRS:
                    if tempProperties > 128 and robotSt.conflictMovingDirectionisForward == True:
                        return False
                    elif tempProperties < 128 and robotSt.conflictMovingDirectionisForward == False:
                        return False
                newSpeed = int(128+((tempProperties-128)*0.5))
                self.SerialMessage = (bytearray([constants.CONST_SERIAL_RPI_SPEED, newSpeed])) # dla bezpieczenstwa
                robotSt.speed = newSpeed
                robotSt.orderSpeed = tempProperties
                return True
            else:
                return False
        elif temp == constants.CONST_INITIALIZE:
            self.SerialMessage = (bytearray([constants.CONST_SERIAL_RPI_INITIALIZED]))
            return True
        elif temp == constants.CONST_STOP:
            rospy.loginfo("Generuje stop")
            self.SerialMessage = (bytearray([constants.CONST_SERIAL_RPI_STOP]))
            return True
        else:
            return False     
class state:
    def __init__(self):
        self.limitSwitches = 15
        self.frontDistance = 0
        self.rearDistance = 0
        self.state = constants.CONST_STATE_MOVEMENT
        self.position = 128
        self.floorSensors = 0
        self.speed = 128
        self.orderPosition = 128
        self.orderSpeed = 128
        self.conflictMovingDirectionisForward = False 

    
    def ChangeState(self,newState):
        self.state = newState
        if newState == constants.CONST_STATE_OBSTACLE or newState == constants.CONST_STATE_STAIRS:
            if self.speed > 128:
                self.conflictMovingDirectionisForward = True
            else:
                self.conflictMovingDirectionisForward = False
    
    def ChangeFrontDistance(self, newDistance):
        self.frontDistance = newDistance
    
    def ChangeRearDistance(self,newDistance):
        self.rearDistance = newDistance
    
    def ChangeLimitSwitches(self,newLimitSwitches):
        self.limitSwitches = newLimitSwitches

    def ChangeFloorSensors(self,newSensors):
        self.floorSensors = newSensors
    
    def ChangeSpeed(self, newSpeed):
        self.speed = newSpeed
    
    def GetLimitSwitches(self):
        return self.limitSwitches

    def GetState(self):
        return self.state
    
    def GetPosition(self):
        return self.position
    
    def GetFrontDistance(self):
        return self.frontDistance

    def GetRearDistance(self):
        return self.rearDistance
    
    def GetSpeed(self):
        return self.speed
    
    def GetFloor(self):
        return self.floorSensors
ser = None
orderObject = None
def receiveOrders(data):
    global ser
    global orderObject


    recieved = data.data
    orderNumber = recieved//1000
    orderProperty = recieved - (orderNumber*1000)
    orderObject.ChangeOrderAndProperties(orderNumber,orderProperty)
    orderObject.previousControlOrder = dt.datetime.now()
    rospy.loginfo("Dostalem-callback")
    rospy.loginfo(recieved)
    # orderObject.ChangeOrder(data.Order)
    # orderObject.ChangeOrderProperties(data.OrderProperty)

    #if data.data =  constants.   
    #ok
def checkTimerElapsed(lastDateTime):
    diff = dt.datetime.now()-lastDateTime
    if diff.total_seconds() > constants.CONST_TIMER:
        return True
    else:
        return False

def talker():
    global ser
    global orderObject

    orderObject = order()
    robotState = state()
    rospy.loginfo("Otwieramy port!")
    ser = serial.Serial()
    ser.port = '/dev/ttyACM0'
    ser.baudrate = 115200
    ser.setDTR(0)
    ser.timeout = 1
    if (ser.isOpen() == False):
        ser.open()
    ser.flushInput()

    ser.reset_input_buffer()
    ser.reset_output_buffer()
    rospy.loginfo("Otwarlismy port!")
    pub = rospy.Publisher('RaspberryControlWriter',String,queue_size=10)
    rospy.init_node('talker',anonymous=True)
    rate = rospy.Rate(20) #10Hz
    time.sleep(20)
    #pub.publish("JEDZIEMY!")
    rospy.loginfo("Jedziemy!")
    rospy.Subscriber("RaspberryControlReader", Int16, receiveOrders)
    ser.write(bytearray([constants.CONST_SERIAL_RPI_INITIALIZED]))
    time.sleep(1)
    bufferSerial = b""
    orderObject.lastDateTime = dt.datetime.now()
    while not rospy.is_shutdown():
        try:

            while orderObject.CheckChange() > 0:
                if orderObject.DecideIfItIsARequest() == True:
                    if orderObject.GenerateResponse(robotState) == True:
                        pub.publish(orderObject.Response)
                elif orderObject.GenerateSerialMessage(robotState) == True:
                    rospy.loginfo(orderObject.ReadSerialMessage())
                    ser.write(orderObject.ReadSerialMessage())
            if checkTimerElapsed(orderObject.previousControlOrder) == True and robotState.state != constants.CONST_STATE_STOP:
                rospy.loginfo("Uplynal czas")
                ser.write(bytearray([constants.CONST_SERIAL_RPI_STOP]))
                robotState.state = constants.CONST_STATE_STOP
            wait = ser.in_waiting
            if wait > 0:
                bufferTemp = ser.read(wait)
                bufferSerial = bufferSerial + bufferTemp 
            while (len(bufferSerial) > 0):
                temp=bufferSerial[0]
                bufferSerial = bufferSerial[1:]
                rospy.loginfo(ord(temp))
                if not temp:
                    function = 0
                else:            
                    function = ord(temp) 
                #rospy.loginfo(function)
                if function == constants.CONST_STATE_MOVEMENT:
                    rospy.loginfo("MOV")
                    robotState.ChangeState(constants.CONST_STATE_MOVEMENT)
                    if orderObject.GenerateResponse(robotState,constants.CONST_READ_STATE) == True:
                        pub.publish(orderObject.Response)
                    os.system('mpg321 /home/pi/Start.mp3 &')
                elif function == constants.CONST_STATE_OBSTACLE:
                    # pub.publish("STATE OBSTACLE")
                    rospy.loginfo("OBST")
                    robotState.ChangeState(constants.CONST_STATE_OBSTACLE)
                    if orderObject.GenerateResponse(robotState,constants.CONST_READ_STATE) == True:
                        pub.publish(orderObject.Response)
                    if(robotState.GetLimitSwitches() < 15):
                        os.system('mpg321 /home/pi/Torture.mp3 &')
                    else:
                        os.system('mpg321 /home/pi/Brake.mp3 &')
                elif function == constants.CONST_STATE_STAIRS:
                    # pub.publish("STATE STAIRS")
                    rospy.loginfo("STAIRS")
                    robotState.ChangeState(constants.CONST_STATE_STAIRS)
                    if orderObject.GenerateResponse(robotState,constants.CONST_READ_STATE) == True:
                        pub.publish(orderObject.Response)
                    os.system('mpg321 /home/pi/Fall.mp3 &')
                elif function == constants.CONST_STATE_STOP:
                    # pub.publish("STATE STOP")
                    rospy.loginfo("STOP")
                    robotState.ChangeState(constants.CONST_STATE_STOP)
                    if orderObject.GenerateResponse(robotState,constants.CONST_READ_STATE) == True:
                        pub.publish(orderObject.Response)
                elif function == constants.CONST_FLOOR:
                    while len(bufferSerial) == 0:
                        bufferTemp = ser.read(ser.in_waiting)
                        bufferSerial = bufferSerial + bufferTemp
                    floorSensors = ord(bufferSerial[0])
                    bufferSerial = bufferSerial[1:]
                    robotState.ChangeFloorSensors(floorSensors)
                    if orderObject.GenerateResponse(robotState,constants.CONST_READ_FLOOR) == True:
                        pub.publish(orderObject.Response)
                    rospy.loginfo(floorSensors)
                elif function == constants.CONST_OBSTACLE_SONAR_BACK:
                    while len(bufferSerial) == 0:
                        bufferTemp = ser.read(ser.in_waiting)
                        bufferSerial = bufferSerial + bufferTemp
                    backSonar = ord(bufferSerial[0])
                    bufferSerial = bufferSerial[1:]
                    robotState.ChangeRearDistance(backSonar)
                    if orderObject.GenerateResponse(robotState,constants.CONST_READ_DISTANCE_REAR) == True:
                        pub.publish(orderObject.Response)
                    rospy.loginfo(backSonar)
                elif function == constants.CONST_OBSTACLE_SONAR_FRONT:
                    while len(bufferSerial) == 0:
                        bufferTemp = ser.read(ser.in_waiting)
                        bufferSerial = bufferSerial + bufferTemp
                    frontSonar = ord(bufferSerial[0])
                    bufferSerial = bufferSerial[1:]
                    robotState.ChangeFrontDistance(frontSonar)
                    if orderObject.GenerateResponse(robotState,constants.CONST_READ_DISTANCE_FRONT) == True:
                        pub.publish(orderObject.Response)
                    rospy.loginfo(frontSonar)
                elif function == constants.CONST_LIMIT:
                    while len(bufferSerial) == 0:
                        bufferTemp = ser.read(ser.in_waiting)
                        bufferSerial = bufferSerial + bufferTemp
                    limitSwitches = ord(bufferSerial[0])
                    bufferSerial = bufferSerial[1:]
                    robotState.ChangeLimitSwitches(limitSwitches)
                    if orderObject.GenerateResponse(robotState,constants.CONST_READ_LIMIT_SWITCHES) == True:
                        pub.publish(orderObject.Response)
                    rospy.loginfo(limitSwitches)
                rate.sleep()
        except rospy.ROSInterruptException:
            ser.close()
            break
    ser.close() 


if __name__ == '__main__':
    talker()
