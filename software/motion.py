import math
import numpy as np
import time
import turtle
import tkinter as t
import struct
import time
from typing import Optional
import serial
from serial.tools import list_ports
class IRobotMotion:
    def open(self):
        pass
    def close(self):
        pass
    def move(self, x_speed, y_speed, rot_speed):
        pass

class SerialPortNotFound(Exception):
    pass

STM_32_HWID = "USB VID:PID=0483:5740"

class OmniMotionRobot(IRobotMotion):
    def __init__(self):
        # Wheel angles
        self.motor_config = [0, 120, 240]

        serial_port: Optional[str] = None
        ports = list_ports.comports()
        devices = {}

        for port, _, hwid in sorted(ports):
            devices[hwid] = port
        
        for hwid in devices.keys():
            if STM_32_HWID in hwid:
                serial_port = devices[hwid]
                break
        
        if serial_port is None:
            raise SerialPortNotFound("Serial port not found")

        self.port = serial.Serial(serial_port, 115200)
            
    
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
            i+=1
        thrower_speed=thrower_speed

        data = struct.pack('<hhhHBH', int(speeds[0]), int(speeds[1]),  int(speeds[2]), thrower_speed, disable_failsafe, 0xAAAA)
        self.port.write(data)
        
    def thrower_control(self,thrower_speed):
        disable_failsafe = 1
        speeds = [0, 0, 0]
        data = struct.pack('<hhhHBH', int(speeds[0]), int(speeds[1]),  int(speeds[2]), thrower_speed, disable_failsafe, 0xAAAA)
        self.port.write(data)

    def stop_robot(self):
        self.move(0,0,0,0)
    
    def speed_limitation(self, speed,direction):
        if direction == 'x':
            if (speed / 10) > 10:
                speed = 10
            else :    
                speed = speed / 10

        elif direction == 'z':
            if (speed / 50) > 3:
                speed = 3  
            elif (speed / 50) < -3 :
                speed = -3
            else :
                speed = speed / 50
        elif direction == 'y':
            if (speed / 50) > 4:
                speed = 4
            elif (speed / 50) < -4:
                speed = -4
            else:
                speed = speed / 50

        return speed


if __name__ == "__main__":
    try:
        x = 412
        speed = 1000
        bot = OmniMotionRobot()
        bot.open()
        bot.move(0,-1,0,100)
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