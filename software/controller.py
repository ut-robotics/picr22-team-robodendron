from inputs import get_gamepad
import math
import threading
import motion
from numpy import interp
from time import sleep
import image_processor
import camera
from follow_ball_depth import *

class XboxController(object):
    # Defining joy interval to normalize output values between -1 and 1
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        # Attributes relative to gamepad
        self.left_joystick_y = 0
        self.left_joystick_x = 0
        self.right_joystick_y = 0
        self.right_joystick_x = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.D_pad_x = 0
        self.D_pad_y = 0
        # Attribute relative to thrower speed
        self.throw_speed = 0
        # Attributes relative to gamepad feedback thread
        self._gamepad_thread = threading.Thread(target=self._monitor_gamepad, args=())
        self._gamepad_thread.daemon = True
        self._gamepad_thread.start()
        

    def read(self): # Return the buttons/joys we care about
        x = self.X
        y = self.Y
        a = self.A
        b = self.B
        ljy = self.left_joystick_y
        ljx = self.left_joystick_x
        rjx = self.right_joystick_x
        dpy = self.D_pad_y
        return [x, y, a, b, ljy, rjx, dpy, ljx]
    
    
    def thrower_speed(self, d_pad_val): # Set the thrower speed incrementally with D-pad (up/down)
        if d_pad_val == -1:
            if self.throw_speed < 1900:
                self.throw_speed += 50
                dist = processedData.depth_frame[processedData.basket_m.y, processedData.basket_m.x]
                print(f"Distance: {dist}, Speed: {self.throw_speed}")
            else:
                pass
        elif d_pad_val == 1:
            if self.throw_speed > 0:
                self.throw_speed += -50
                print(self.throw_speed)
            else:
                pass
    

    def throw(self):# Launch shot sequence to test current thrower speed
        self.A = 0
        bot.move(8,0,0,self.throw_speed) 
        sleep(0.8)


    def _monitor_gamepad(self): # Gamepad thread function that updates the gamepad buttons' state
        joy_threshold = 0.1

        while True:

            events = get_gamepad()

            for event in events:

                if event.code == 'BTN_SOUTH': # Button A
                    self.A = event.state

                elif event.code == 'BTN_NORTH': # Button X /!\ X and Y are switched in the lib...
                    self.X = event.state

                elif event.code == 'BTN_WEST': # Button Y
                    self.Y = event.state

                elif event.code == 'BTN_EAST': # Button B
                    self.B = event.state

                elif event.code == 'ABS_HAT0X': # D-pad Y-axis
                    self.D_pad_x = event.state

                elif event.code == 'ABS_HAT0Y': # D-pad X-axis
                    self.D_pad_y = event.state
                    self.thrower_speed(event.state)

                elif event.code == 'ABS_Y': # Left Joy Y-axis
                    joy_val_LY = event.state / XboxController.MAX_JOY_VAL  # normalize between -1 and 1
                    if joy_val_LY < - joy_threshold or joy_val_LY > joy_threshold: # Set tiny joy values to 0
                        self.left_joystick_y = joy_val_LY
                    else:
                        self.left_joystick_y = 0

                elif event.code == 'ABS_X': # Left Joy X-axis
                    joy_val_LX = event.state / XboxController.MAX_JOY_VAL  # normalize between -1 and 1
                    if joy_val_LX < - joy_threshold or joy_val_LX > joy_threshold: # Set tiny joy values to 0
                        self.left_joystick_x = joy_val_LX
                    else:
                        self.left_joystick_x = 0
                
                elif event.code == 'ABS_RX': # Right Joy X-axis
                    joy_val_X = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                    if joy_val_X < - joy_threshold or joy_val_X > joy_threshold: # Set tiny joy values to 0
                        self.right_joystick_x = joy_val_X
                    else:
                        self.right_joystick_x = 0
    

if __name__ == '__main__':
    joy = XboxController()
    bot = motion.OmniMotionRobot()
    cam = camera.OpenCVCamera(id = 2)
    cam = camera.RealsenseCamera(exposure = 100)
    processor = image_processor.ImageProcessor(cam, debug=True)
    processor.start()
    bot.open()
    try:
        while True:
            processedData = processor.process_frame(aligned_depth=False)
            #print(joy.read())
            x_speed = interp(joy.read()[4],[-1,1],[10,-10]) # Map joy value to x-speed
            y_speed = interp(joy.read()[7],[-1,1],[-10,10]) # Map joy value to y-speed
            rot_speed = interp(joy.read()[5],[-1,1],[3,-3]) # Map joy value to rot-speed
            
            if joy.read()[2] == 1: # If A is pressed launch throwing sequence
                joy.throw()
            
            if joy.read()[1] == 1: # If A is pressed launch throwing sequence
                main_loop(True)

            if joy.read()[3] == 1: # If B is pressed stop all motors and kill gamepad control (Emergency Switch)
                main_loop(False)
                bot.move(0, 0, 0, 0)
                bot.close
                exit()
            
            bot.move(x_speed, y_speed, rot_speed, joy.throw_speed) # Send speed values to robot
            
    except KeyboardInterrupt : # Close script with a CTRL+C + Stop robot
        bot.move(0, 0, 0, 0)
        bot.close()