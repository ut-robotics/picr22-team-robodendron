import image_processor
import camera
import motion
import cv2
import time

def main_loop():
    debug = True
    
    motion_sim = motion.TurtleRobot()
    motion_sim2 = motion.TurtleOmniRobot()
    motion_sim3 = motion.OmniMotionRobot()
    
    #camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100)
    
    processor = image_processor.ImageProcessor(cam, debug=debug)

    processor.start()
    motion_sim.open()
    motion_sim2.open()
    motion_sim3.open()

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    try:
        while True:
            
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=False)
            largest = max(processedData.balls, key = lambda ball: ball.size, default=None)
            if(largest):
                cv2.circle(processedData.debug_frame,(largest.x, largest.y), 20, (255, 0, 255), -1)
                print(processedData.debug_frame.shape[1]/2 - largest.x)
                speedx = (largest.x - processedData.debug_frame.shape[1]/2) / 60
                speedy = (2*processedData.debug_frame.shape[0]/3 - largest.y if 2*processedData.debug_frame.shape[0]/3 - largest.y > 0 else 0)
                
                speedx = speedx / 100
                speedy = speedy / 100

                motion_sim.move(0, speedy, speedx)
                motion_sim2.move(0, speedy, speedx)
                motion_sim3.move(0, speedy, speedx)
            print(largest)
            

            # This is where you add the driving behaviour of your robot. It should be able to filter out
            # objects of interest and calculate the required motion for reaching the objects

            frame_cnt +=1

            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                print("FPS: {}, framecount: {}".format(fps, frame_cnt))
                print("ball_count: {}".format(len(processedData.balls)))

                #if (frame_cnt > 1000):
                #    break

            if debug:
                debug_frame = processedData.debug_frame

                cv2.imshow('debug', debug_frame)

                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break
    except KeyboardInterrupt:
        print("closing....")
    finally:
        cv2.destroyAllWindows()
        processor.stop()
        motion_sim.close()
        motion_sim2.close()
        motion_sim3.close()

main_loop()
