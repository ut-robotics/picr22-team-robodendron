from inputs import get_gamepad
import math
import threading
import motion
from numpy import interp
from time import sleep
import image_processor
import camera

class XboxController(object):
    # Defining joy interval to normalize output values between -1 and 1
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        # Attributes relative to gamepad
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.DPadX = 0
        self.DPadY = 0
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
        ljy = self.LeftJoystickY
        rjx = self.RightJoystickX
        dpy = self.DPadY
        return [x, y, a, b, ljy, rjx, dpy]
    
    
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
                    self.DPadX = event.state

                elif event.code == 'ABS_HAT0Y': # D-pad X-axis
                    self.DPadY = event.state
                    self.thrower_speed(event.state)

                elif event.code == 'ABS_Y': # Left Joy Y-axis
                    joy_val_Y = event.state / XboxController.MAX_JOY_VAL  # normalize between -1 and 1
                    if joy_val_Y < - joy_threshold or joy_val_Y > joy_threshold: # Set tiny joy values to 0
                        self.LeftJoystickY = joy_val_Y
                    else:
                        self.LeftJoystickY = 0
                
                elif event.code == 'ABS_RX': # Right Joy X-axis
                    joy_val_X = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                    if joy_val_X < - joy_threshold or joy_val_X > joy_threshold: # Set tiny joy values to 0
                        self.RightJoystickX = joy_val_X
                    else:
                        self.RightJoystickX = 0
    

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
            rot_speed = interp(joy.read()[5],[-1,1],[3,-3]) # Map joy value to rot-speed
            
            if joy.read()[2] == 1: # If A is pressed launch throwing sequence
                joy.throw()

            if joy.read()[3] == 1: # If B is pressed kill stop all motors and kill gamepad control (Emergency Switch)
                bot.move(0, 0, 0, 0)
                bot.close
                exit()
            
            bot.move(x_speed, 0, rot_speed, joy.throw_speed) # Send speed values to robot
            
    except KeyboardInterrupt : # Close script with a CTRL+C + Stop robot
        bot.move(0, 0, 0, 0)
        bot.close()