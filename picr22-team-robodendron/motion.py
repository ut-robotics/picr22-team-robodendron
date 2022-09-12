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

class TurtleRobot(IRobotMotion):
    def __init__(self, name="Default turtle robot"):

        window = tk.Tk()
        window.title(name)

        canvas = tk.Canvas(master=window, width=500, height=500)
        canvas.pack()

        self.screen = turtle.TurtleScreen(canvas)
        self.turtle_obj = turtle.RawTurtle(self.screen)
        self.turtle_obj.speed('fastest')

        self.steps = 20

    def open(self):
        print("Wroom! Starting up turtle!")

    def close(self):
        print("Going to dissapear...")

    #Very dumb logic to draw motion using turtle
    def move(self, x_speed, y_speed, rot_speed):
        self.screen.tracer(0, 0)
        angle_deg = 0

        angle_deg = np.degrees(math.atan2(x_speed, y_speed))

        distance = math.sqrt(math.pow(x_speed, 2) + math.pow(y_speed, 2))

        distance_step = distance / float(self.steps)
        angel_step = np.degrees(rot_speed / float(self.steps))

        self.turtle_obj.penup()
        self.turtle_obj.reset()
        self.turtle_obj.right(angle_deg - 90)
        self.turtle_obj.pendown()

        for i in range(0, self.steps):
            self.turtle_obj.right(angel_step)
            self.turtle_obj.forward(distance_step)

        self.turtle_obj.penup()
        self.screen.update()

class TurtleOmniRobot(TurtleRobot):
    def __init__(self, name="Default turtle omni robot"):
        TurtleRobot.__init__(self, name)

        # Wheel angles
        self.motor_config = [0, 120, 240]

    def move(self, x_speed, y_speed, rot_speed):
        speeds = [0, 0, 0]

        # This is where you need to calculate the speeds for robot motors
        robotSpeed = math.sqrt(x_speed * x_speed + y_speed * y_speed)
        robotDirectionAngle = math.atan2(x_speed, y_speed)

        i = 0
        for wheelAngle in self.motor_config:
            wheelLinearVelocity = robotSpeed * math.cos(robotDirectionAngle - wheelAngle/180*math.pi) + rot_speed
            #wheelSpeedToMainboardUnits = gearboxReductionRatio * encoderEdgesPerMotorRevolution / (2 * math.pi * wheelRadius * pidControlFrequency)
            #wheelAngularSpeedMainboardUnits = wheelLinearVelocity * wheelSpeedToMainboardUnits
            speeds[i] = wheelLinearVelocity
            print("speeed omnii #######################")
            print(speeds[i])
            i+=1

        simulated_speeds = self.speeds_to_direction(speeds)

        TurtleRobot.move(self, speeds[0], speeds[1], speeds[2])

    def speeds_to_direction(self, speeds):
        offset_x = 0
        offset_y = 0
        degree = int((speeds[0] + speeds[1] + speeds[2]) / 3)
    
        for i in range(0, 3):
            end_vector = self.motor_side_forward_scale(self.motor_config[i] + 90, speeds[i], offset_x, offset_y)
            print(end_vector)
            offset_x = end_vector[0]
            offset_y = end_vector[1]
    
        offsets = [offset_x * -1, offset_y]
        speeds = [int(a / 1.5) for a in offsets]
        speeds.append(degree)
    
        return speeds
 
    def motor_side_forward_scale(self, angel, length, offset_x=0, offset_y=0):
        ang_rad = math.radians(angel)
        return [length * math.cos(ang_rad) + offset_x, length * math.sin(ang_rad) + offset_y]


class OmniMotionRobot(IRobotMotion):
    def __init__(self):
        # Wheel angles
        self.motor_config = [0, 120, 240]
        self.port = None
    
    
    def open(self):
        self.port = serial.Serial('/dev/ttyACM0')  # open serial port
        print(self.port.name)         # check which port was really used
        #ser.write(b'hello')     # write a string
    
    
    def close(self):
        self.port.close() # close port
    
    
    def move(self, x_speed, y_speed, rot_speed):
        disable_failsafe = 1
        speeds = [0, 0, 0]
        robotSpeed = math.sqrt(x_speed * x_speed + y_speed * y_speed)
        robotDirectionAngle = math.atan2(x_speed, y_speed)

        i = 0
        for wheelAngle in self.motor_config:
            wheelLinearVelocity = robotSpeed * math.cos(robotDirectionAngle - wheelAngle/180*math.pi) + rot_speed
            #wheelSpeedToMainboardUnits = gearboxReductionRatio * encoderEdgesPerMotorRevolution / (2 * math.pi * wheelRadius * pidControlFrequency)
            #wheelAngularSpeedMainboardUnits = wheelLinearVelocity * wheelSpeedToMainboardUnits
            speeds[i] = wheelLinearVelocity
            print(speeds[i])
            i+=1
        thrower_speed=0
        # if(speeds[0] > 10): speeds[0] = 10
        # if(speeds[1] > 10): speeds[1] = 10
        # if(speeds[2] > 10): speeds[2] = 10
        
        # speeds[0] = 10
        # speeds[1] = 10
        # speeds[2] = 10
        #data = struct.pack('<hhhHBH', 20, 20, 20, thrower_speed, disable_failsafe, 0xAAAA)
        data = struct.pack('<hhhHBH', int(speeds[0]), int(speeds[1]),  int(speeds[2]), thrower_speed, disable_failsafe, 0xAAAA)
        
        self.port.write(data)
        print("ok")

if __name__ == "__main__":
    bot = OmniMotionRobot()
    bot.open()
    bot.move(10, 20, 20)

    time.sleep(2)
    bot.move(0, 0, 0)
    bot.close()
