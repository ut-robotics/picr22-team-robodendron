import image_processor
import camera
import motion
import cv2
import time
import threading
import referee_command
import json
import pandas as pd
import numpy as np
import math
from realsense_depth import *

exitFlag = 0
global speedz
global speedx
global state
state = 'find_ball'

speed_df = pd.read_csv('speed_magenta.csv')



def speed_thrower(dist):
    
    #speed = min(int(0.316*dist + 450),1900)

    speed = min(int(0.330*dist + 415),1900)

    """
    if dist <= 1500:
        speed = int(0.316*dist + 450)
   
    elif dist > 1500 and dist < 2300:
        speed = int(0.316*dist + 450)

    elif dist >= 2300:
       speed = int(0.316*dist + 450)
    """
    return speed

def speed_limitation(speed,direction):
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

class controller(threading.Thread):
   
    def __init__(self, threadID, name,processor):
        
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.processor = processor

    def run(self):
        print("Starting " + self.name)

    def control_on(self):
        global speedz
        global speedx
        global motion_sim3 
        global state
        self.processedData = self.processor.process_frame(aligned_depth=False)
        largest = max(self.processedData.balls , key = lambda ball: ball.size, default=None)
        if largest:
            if state == 'find_ball':
                state = 'go_ball'
            #cv2.circle(self.processedData.debug_frame,(largest.x, largest.y), 20, (255, 0, 255), -1)
            if self.name == 'z':
                speedz= (self.processedData.debug_frame.shape[1]/2) - largest.x 
                speedz = speed_limitation(speedz,'z')

            elif self.name == 'x':
                speedx = (2*self.processedData.debug_frame.shape[0]/3 - largest.y if 2*self.processedData.debug_frame.shape[0]/3 - largest.y > 0 else 0)
                speedx = speed_limitation(speedx,'x')

                speedy = 0
                
                try:
                    print('speedz: ',speedz)
                    print('speedx: ', speedx)
                    if state == 'go_ball':
                        if speedx < 0.02: 
                            speedx = 0
                            speedz = 0
                            state = 'align_basket'
                            #print(state)
                      
                        motion_sim3.move(speedx, speedy, speedz,100)

                except:
                    print('except')
                    motion_sim3.move(0, 0, 0,100)
                    state = 'find_ball'

        else : 
            state = 'find_ball'


class Scanning_ball(threading.Thread):

    def __init__(self,name,processor):
        threading.Thread.__init__(self)
        self.name = name
        self.stop = False
        self.processor = processor
        self.count = 0
        self.basket_a = None

    def run(self):
        print("Starting " + self.name)

    def scan_ball(self):
        global state 
        global oponent_basket
        global target_basket
        processedData = self.processor.process_frame(aligned_depth=False)
        
        if target_basket == 'blue':
            direction = 1
        else:
            direction = -1
        
        if state == 'find_ball':
            motion_sim3.move(0, 0, direction * 3,100)
            # time.sleep(0.2)
            # motion_sim3.move(0, 0, 0,100)
            # time.sleep(0.2)
            self.count+=1

                # print(self.count)
                # if oponent_basket == 'blue':
                #     basket_ = processedData.basket_b
                # else:
                #     basket_ = processedData.basket_m

                # try:    
                #     print(basket_)
                #     if bakset_.exists :
                #         print('go to basket')
                #         motion_sim3.move(3, 0, basket_.x/100,100)
                #         time.sleep(0.6)
                #         self.count = 0
                #     else: 
                #         motion_sim3.move_turn()
                #         time.sleep(0.2)
                #         motion_sim3.move(0, 0, 0,100)
                #         time.sleep(0.6)
                # except: 
                #     motion_sim3.move_turn()
                #     time.sleep(0.2)
                #     motion_sim3.move(0, 0, 0,100)
                #     time.sleep(0.6)

class referee_Thread(threading.Thread):

    def __init__(self,name):
        threading.Thread.__init__(self)
        self.rf_command = referee_command.referee_command()
        self.name = name
        self.interupt = True
        self.stop = False

    def run(self):
        global target_basket
        global oponent_basket
        global state 
        print("Starting " + self.name)
        try:
            self.rf_command.connect()
        except:
            print("can not connect to server")
            self.interupt = True
        
        while True:
            msg = json.loads(self.rf_command.listen())
            if msg.get("signal") == "stop":
                self.interupt = True
                state = 'find_ball'
                print(msg.get("signal"))

            else:
                for robot in range(len(msg.get("targets"))):
                    if msg.get("targets")[robot] == "robodendron":
                        target_basket = msg.get("baskets")[robot]
                        if target_basket == 'blue':
                            oponent_basket = 'magenta'
                        else:
                            oponent_basket = "blue"

                        print(msg.get("signal"),"bakset target : ",target_basket)
                        self.interupt = False


class align_control(threading.Thread):
   
    def __init__(self, threadID, name,processor):
        
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.processor = processor
    
    def run(self):
        print("Starting " + self.name)
    
    
    def throw(self, depth_frame):
        distance = depth_frame[self.basket_.y, self.basket_.x]
        print("------------------------------------")
        print(distance)
        print("------------------------------------")
        time.sleep(0.2)
        motion_sim3.thrower_control(600)
        time.sleep(0.2)
        print('dist basket',self.basket_.y)
        thr_speed = speed_thrower(distance)
        print('speed',thr_speed)
        motion_sim3.move(6,0,0,thr_speed) 
        time.sleep(1)
        motion_sim3.thrower_control(300)


    def align_on(self):
        global state
        processedData = self.processor.process_frame(aligned_depth=False)
        if state == 'align_basket':
            largest = max(processedData.balls , key = lambda ball: ball.size, default=None)
            if target_basket == 'blue':
                self.basket_ = processedData.basket_b
            else:
                self.basket_ = processedData.basket_m
            
            if largest:
                #print(abs(self.basket_.x))
    
                speedz = (processedData.debug_frame.shape[1]/2) - largest.x 
                speedx = (2*processedData.debug_frame.shape[0]/3 - largest.y if 2*processedData.debug_frame.shape[0]/3 - largest.y > 0 else 0)
                speedy = largest.x  - self.basket_.x

                speedx = speed_limitation(speedx,'x')
                speedz = speed_limitation(speedz,'z')
                speedy = speed_limitation(speedy,'y')
                
                print('aligning.....',abs(speedy))

                if abs(speedy) < 0.10:
                    speedz =  0
                    speedx =  0

                    state = 'shoot'

                
                motion_sim3.move(speedx, speedy ,speedz,100)

            # else:
            #     state = 'find_ball'

        elif state == 'shoot':
            self.throw(processedData.depth_frame)
            state = 'find_ball'



    

def main_loop():
    global speedz
    global speedx
    global motion_sim3 
    global state 
    global target_basket


    debug = True
    
    #motion_sim = motion.TurtleRobot()
    #motion_sim2 = motion.TurtleOmniRobot()
    motion_sim3 = motion.OmniMotionRobot()
    
    #camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100)
    
    processor = image_processor.ImageProcessor(cam, debug=debug)

    processor.start()
    #motion_sim.open()
    #motion_sim2.open()
    motion_sim3.open()

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0

    processedData = processor.process_frame(aligned_depth=False)
    referee_thread = referee_Thread("referee_thread")
    referee_thread.start()
    thread1 = controller(1, "z",processor)
    thread2 = controller(2, "x", processor)
    align_control1 = align_control(1,"align_basket_controller",processor)
    scanning_ball = Scanning_ball('scan_ball',processor)

    thread1.start()
    thread2.start()
    align_control1.start()
    scanning_ball.start()
    motion_sim3.move(0, 0, 0,100)

    try:
        while True:
            state = 'find_ball'
            while not referee_thread.interupt:
                print("state == ",state)
                
                thread1.control_on()
                thread2.control_on()
                
                scanning_ball.scan_ball()
                align_control1.align_on()

                frame_cnt +=1

                frame += 1
                if frame % 30 == 0:
                    frame = 0
                    end = time.time()
                    fps = 30 / (end - start)
                    start = end
                    # print("FPS: {}, framecount: {}".format(fps, frame_cnt))
                    # print("ball_count: {}".format(len(processedData.balls)))

                    #if (frame_cnt > 1000):
                    #    break

                if debug:
                    debug_frame = processedData.debug_frame

                    #cv2.imshow('debug', debug_frame)

                    k = cv2.waitKey(1) & 0xff
                    if k == ord('q'):
                        break
            
            motion_sim3.stop_robot()

    except KeyboardInterrupt:
        motion_sim3.stop_robot()
        print("closing....")
        

    finally:
        #cv2.destroyAllWindows()
        processor.stop()
        #motion_sim.close()
        #motion_sim2.close()
        motion_sim3.close()

main_loop()
