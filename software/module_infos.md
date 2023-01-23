This is an example implementation of a LUT based segmentation module that can be used with Realsense cameras.

Designed to use Python 3. Dependencies: numpy, openCV, pyrealsense2

This code is heavily inspired by [Mitupead - football robot](https://github.com/lwd8cmd/Mitupead)

## Modules in the code

### Color.py
Contains data of the colors that are detected. Also contains data used to display the colors in the debug view.

### camera.py
Contains a ICamera interface that describes methods required by the image processing module. Also contains an example implementation for Intel RealSense cameras (RealsenseCamera) and OpenCV web cameras (OpenCVCamera).

### image_processor.py 
Main module for image processing. Is responsible for image color and feature segmentation. Object analysis and filtration should happen in this module.
```
def process_frame(self, aligned_depth = False) -> ProcessedResults: 
```
Is the main method responsible for providing processed frame data.

### motion.py
Contains a IRobotMotion interface that descibes method required for moving the robot. Also contains an example implementation (TurtleRobot) that visualizes motion using turtle tools in python.


### config_colors.py
Utility to configure colors. Check log for detailed instructions.

### referee_command.py
Script that connects to the referee server to start and stop the robot's sequence.

### controller.py
Script that enables Gamepad support to control the robot manually. Useful for debuging and throw speed
measurements.

### follow_ball_depth.py
Main script that drives the robot. This is where we track balls, go towards them and initiate throws. This final version relies on the camera's depth sensor, therefore its name.

## How to use

Segmentation module installation:
```
cd segment_module
pip3.9 install .
```

Running color configurator:
```
mkdir colors
touch colors/colors.pkl
python3.9 config_colors.py
```

Starting the robot's main sequence (requires to setup the referee server):
```
python3.9 follow_ball_depth.py
```

If you encounter dependency errors, resolve them with pip3.9
