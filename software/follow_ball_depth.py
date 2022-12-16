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
import sys
from enum import Enum

class states(Enum):
        find_ball = 1
        go_ball = 2
        align_basket = 3
        shoot = 4
        
class robot():

    def __init__(self):
        robot.speedz = None
        robot.speedx = None
        robot.motion_sim3 = motion.OmniMotionRobot()
        robot.state = states.find_ball
        robot.oponent_basket = None
        robot.target_basket = None



    class controller(threading.Thread):
    
        def __init__(self, threadID, name,processor):
            
            threading.Thread.__init__(self)
            self.threadID = threadID
            self.name = name
            self.processor = processor

        def run(self):
            print("Starting " + self.name)

        def control_on(self):
            self.processedData = self.processor.process_frame(aligned_depth=False)
            largest = max(self.processedData.balls , key = lambda ball: ball.size, default=None)
            if largest:
                if robot.state == states.find_ball:
                    robot.state = states.go_ball
                #cv2.circle(self.processedData.debug_frame,(largest.x, largest.y), 20, (255, 0, 255), -1)
                if self.name == 'z':
                    robot.speedz= (self.processedData.debug_frame.shape[1]/2) - largest.x
                    robot.speedz = robot.motion_sim3.speed_limitation(robot.speedz,'z')

                elif self.name == 'x':
                    robot.speedx = (2*self.processedData.debug_frame.shape[0]/3 - largest.y if 2*self.processedData.debug_frame.shape[0]/3 - largest.y > 0 else 0)
                    robot.speedx = robot.motion_sim3.speed_limitation(robot.speedx,'x')

                    speedy = 0
                    
                    try:
                        print('robot.speedz: ',robot.speedz)
                        print('robot.speedx: ', robot.speedx)
                        if robot.state == states.go_ball:
                            if robot.speedx < 0.02: 
                                robot.speedx = 0
                                robot.speedz = 0
                                robot.state = states.align_basket
                                #print(robot.state)
                        
                            robot.motion_sim3.move(robot.speedx, speedy, robot.speedz,100)

                    except:
                        print('except')
                        robot.motion_sim3.move(0, 0, 0,100)
                        robot.state = states.find_ball

            else : 
                robot.state = states.find_ball


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
         
            processedData = self.processor.process_frame(aligned_depth=False)
            
            if robot.target_basket == 'blue':
                direction = 1
            else:
                direction = -1
            
            if robot.state == states.find_ball:
                robot.motion_sim3.move(0, 0, direction * 3,100)
                # time.sleep(0.2)
                # robot.motion_sim3.move(0, 0, 0,100)
                # time.sleep(0.2)
                self.count+=1

                    # print(self.count)
                    # if robot.oponent_basket == 'blue':
                    #     basket_ = processedData.basket_b
                    # else:
                    #     basket_ = processedData.basket_m

                    # try:    
                    #     print(basket_)
                    #     if bakset_.exists :
                    #         print('go to basket')
                    #         robot.motion_sim3.move(3, 0, basket_.x/100,100)
                    #         time.sleep(0.6)
                    #         self.count = 0
                    #     else: 
                    #         robot.motion_sim3.move_turn()
                    #         time.sleep(0.2)
                    #         robot.motion_sim3.move(0, 0, 0,100)
                    #         time.sleep(0.6)
                    # except: 
                    #     robot.motion_sim3.move_turn()
                    #     time.sleep(0.2)
                    #     robot.motion_sim3.move(0, 0, 0,100)
                    #     time.sleep(0.6)

    class referee_Thread(threading.Thread):

        def __init__(self,name):
            threading.Thread.__init__(self)
            self.rf_command = referee_command.referee_command()
            self.name = name
            self.interupt = True
            self.stop = False

        def run(self):
            print("Starting " + self.name)
            try:
                self.rf_command.connect()
            except:
                print("can not connect to server")
                self.interupt = True
            
            while True:

                msg = json.loads(self.rf_command.listen())
                print(msg)
                try :
                    index_robot = msg.get("targets").index("robodendron") 
                except:
                    break
                if msg.get("targets")[index_robot] == "robodendron":
                    if msg.get("signal") == "stop":
                        self.interupt = True
                        robot.state = states.find_ball
                        print(msg.get("signal"))

                    else:
                        robot.target_basket = msg.get("baskets")[index_robot]
                        if robot.target_basket == 'blue':
                            robot.oponent_basket = 'magenta'
                        else:
                            robot.oponent_basket = "blue"

                        print(msg.get("signal"),"bakset target : ",robot.target_basket)
                        self.interupt = False


    class align_control(threading.Thread):
    
        def __init__(self, threadID, name,processor):
            
            threading.Thread.__init__(self)
            self.threadID = threadID
            self.name = name
            self.processor = processor
        
        def run(self):
            print("Starting " + self.name)

        def speed_thrower(self,dist):
            speed = min(int(0.230*dist + 445),1900)
            return speed
        
        
        def throw(self, depth_frame):
            distance = depth_frame[self.basket_.y, self.basket_.x]
            print("------------------------------------")
            print(distance)
            print("------------------------------------")
            time.sleep(0.2)
            robot.motion_sim3.thrower_control(600)
            time.sleep(0.2)
            print('dist basket',self.basket_.y)
            thr_speed = self.speed_thrower(distance)
            print('speed',thr_speed)
            robot.motion_sim3.move(6,-0.15,0,thr_speed) 
            time.sleep(1)
            robot.motion_sim3.thrower_control(300)


        def align_on(self):
            processedData = self.processor.process_frame(aligned_depth=False)
            if robot.state == states.align_basket:
                largest = max(processedData.balls , key = lambda ball: ball.size, default=None)
                if robot.target_basket == 'blue':
                    self.basket_ = processedData.basket_b
                else:
                    self.basket_ = processedData.basket_m
                
                if largest:
                    #print(abs(self.basket_.x))
        
                    robot.speedz = (processedData.debug_frame.shape[1]/2) - largest.x 
                    robot.speedx = (2*processedData.debug_frame.shape[0]/3 - largest.y if 2*processedData.debug_frame.shape[0]/3 - largest.y > 0 else 0)
                    speedy = largest.x  - self.basket_.x

                    robot.speedx = robot.motion_sim3.speed_limitation(robot.speedx,'x')
                    speedy = robot.motion_sim3.speed_limitation(speedy,'y')
                    robot.speedz = robot.motion_sim3.speed_limitation(robot.speedz,'z')
                    
                    print('aligning.....',abs(speedy))

                    if abs(largest.x  - self.basket_.x) < 3:
                        robot.speedz =  0
                        robot.speedx =  0

                        robot.state = states.shoot

                    
                    robot.motion_sim3.move(robot.speedx, speedy ,robot.speedz,100)

                # else:
                #     robot.state = 'find_ball'

            elif robot.state == states.shoot:
                self.throw(processedData.depth_frame)
                robot.state = states.find_ball



    

def main_loop(method_controller=True):

    robodendron_robot = robot()

    debug = True
    
    if method_controller == False:
        sys.exit()
    #motion_sim = motion.TurtleRobot()
    #motion_sim2 = motion.TurtleOmniRobot()
    robodendron_robot.motion_sim3 = motion.OmniMotionRobot()
    
    #camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100)
    
    processor = image_processor.ImageProcessor(cam, debug=debug)

    processor.start()
    #motion_sim.open()
    #motion_sim2.open()
    robodendron_robot.motion_sim3.open()

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0

    processedData = processor.process_frame(aligned_depth=False)
    referee_thread = robodendron_robot.referee_Thread("referee_thread")
    referee_thread.start()
    thread1 = robodendron_robot.controller(1, "z",processor)
    thread2 = robodendron_robot.controller(2, "x", processor)
    align_control1 = robodendron_robot.align_control(1,"align_basket_controller",processor)
    scanning_ball = robodendron_robot.Scanning_ball('scan_ball',processor)

    thread1.start()
    thread2.start()
    align_control1.start()
    scanning_ball.start()
    robodendron_robot.motion_sim3.move(0, 0, 0,100)

    try:
        while True:
            state = states.find_ball
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
            
            robodendron_robot.motion_sim3.stop_robot()

    except KeyboardInterrupt:
        robodendron_robot.motion_sim3.stop_robot()
        print("closing....")
        

    finally:
        #cv2.destroyAllWindows()
        processor.stop()
        #motion_sim.close()
        #motion_sim2.close()
        robodendron_robot.motion_sim3.close()

if __name__ == "__main__":
    main_loop()
