#import turtle
import math
import numpy as np
import time
import turtle
import tkinter as tk
import serial
import struct
import time

class IRobotMotion:
    def open(self):
        pass
    def close(self):
        pass
    def move(self, x_speed, y_speed, rot_speed):
        pass


class OmniMotionRobot(IRobotMotion):
    def __init__(self):
        # Wheel angles
        self.motor_config = [0, 120, 240]
        self.port = None
    
    
    def open(self):

        ## Create a loop that scan all the port
        try:
            self.port = serial.Serial('/dev/ttyACM0')  # open serial port
        except:
            self.port = serial.Serial('/dev/ttyACM1')  # open serial port
        print(self.port.name)         # check which port was really used
        #ser.write(b'hello')     # write a string
    
    
    def close(self):
        self.port.close() # close port
    
    
    def move(self, x_speed, y_speed, rot_speed,thrower_speed = 0):
        disable_failsafe = 1
        speeds = [0, 0, 0]
        robotSpeed = math.sqrt(x_speed * x_speed + y_speed * y_speed)
        robotDirectionAngle = math.atan2(x_speed, y_speed)

        i = 0

        gearboxReductionRatio = 1
        encoderEdgesPerMotorRevolution = 64
        wheelRadius = 0.035
        pidControlFrequency = 60

        for wheelAngle in self.motor_config:
            wheelLinearVelocity = robotSpeed * math.cos(robotDirectionAngle - wheelAngle/180*math.pi) + rot_speed
            wheelSpeedToMainboardUnits = gearboxReductionRatio * encoderEdgesPerMotorRevolution / (2 * math.pi * wheelRadius * pidControlFrequency)
            wheelAngularSpeedMainboardUnits = wheelLinearVelocity * wheelSpeedToMainboardUnits
            speeds[i] = wheelAngularSpeedMainboardUnits
            #print(speeds[i])
            i+=1
        thrower_speed=thrower_speed
        # if(speeds[0] > 10): speeds[0] = 10
        # if(speeds[1] > 10): speeds[1] = 10
        # if(speeds[2] > 10): speeds[2] = 10
        
        # speeds[0] = 10
        # speeds[1] = 10
        # speeds[2] = 10
        #data = struct.pack('<hhhHBH', 20, 0, 0, thrower_speed, disable_failsafe, 0xAAAA)
        data = struct.pack('<hhhHBH', int(speeds[0]), int(speeds[1]),  int(speeds[2]), thrower_speed, disable_failsafe, 0xAAAA)
        #print(speeds)
        self.port.write(data)
        
    def thrower_control(self,thrower_speed):
        disable_failsafe = 1
        speeds = [0, 0, 0]
        data = struct.pack('<hhhHBH', int(speeds[0]), int(speeds[1]),  int(speeds[2]), thrower_speed, disable_failsafe, 0xAAAA)
        self.port.write(data)
        

    def move_straight(self):
        speed_x = 3  
        speed_y = 0
        speed_z = 0
        self.move(speed_x,speed_y,speed_z)

        return 0 
    
    def move_turn(self):
        speed_x = 0  
        speed_y = 0
        speed_z = 10
        self.move(speed_x,speed_y,speed_z,100)
        return 0 

    def move_circle(self):
        speedz = 0
        speedy = 0

        self.move(speedx, speedy, speedz)
        print("----------------",speedx,speedz)
    
        speedx = 0
    def stop_robot(self):
        self.move(0,0,0)


if __name__ == "__main__":
    try:
        x = 412
        speed = 1000
        bot = OmniMotionRobot()
        bot.open()
        bot.move(0,0,1,100)
        # bot.thrower_control(300)
        # time.sleep(0.5)
        # bot.thrower_control(speed)
        time.sleep(10)
        # bot.thrower_control(300)
        # time.sleep(0.5)
        bot.stop_robot()

    except KeyboardInterrupt:
        bot.thrower_control(300)
        time.sleep(0.5)
        bot.stop_robot()
        bot.thrower_control(100)