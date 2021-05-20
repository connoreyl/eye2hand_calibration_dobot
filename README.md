# HAND-TO-EYE CALIBRATION WITH THE DOBOT MAGICIAN


## By Connor Eyles, Joshua Poot and Travis Goodshaw


### Project Outline
The code included in this repository is for calibrating the position of the objects and robot base with respect to the camera. 
The hand to eye calibration is done by finding the positions of the AR Tags using the ar_tag_toolbox and an RGB-D Camera.
The AR Tag for the base is transformed by multiplying the transform offset and inverting this to turn it from the camera frame into the robot frame of reference.
The GUI allows us to connect, disconnect and initialise to the Dobot. It offers a user the functionality to calibrate the tag positions with respect to the camera, move the dobot to each tag position and switch to tag follow mode. The GUI also displays the X,Y and Z position of the tags with respect to the robot. 
If the tags, robot or camera have been moved, the GUI also allows for recalibration without having to reinitialise the entire program.
If required, the transforms of all the Camera, the AR Tags, the Robot Base and the Robot End Effector can be plotted in a graph.

### Project Functionalities
The program has four primary functions:
- Calibration: The camera will determine the poses of the AR Tags, and will determine the transforms of the camera-to-base and tags-to base. Using this, the camera can be placed at any point in the 3D space, even upside down and behind the robot, provided that it can see the tag at the base of the robot.
- Moving to Tags: The Dobot will move to any of the calibrated AR Tags in visual range. The GUI allows the user to select which tag it can move to, but will not move to tags that aren't in range.
- Follow Mode: The Dobot will follow a single AR Tag, even if the AR Tag is moved but still in range of the sensor.
- Plot Tags: The program will generate a 3D plot of the transforms of the camera, end-effector and tags with respect to the robot base.

### Required ROS Packages
To run our program, the following ROS packages are required:
- ar_track_alvar
- ar_tag_toolbox
- dobot_magician_driver
- a usb camera driver (in this case, we used the realsense-ros driver)

### Other Notes

As a note: the ar_convert ros package is authored By Himavaan Chandra. As a group, we had trouble accessing the AR Tag topic in Matlab for an unknown reason, and this package allowed us to circumvent this problem, allowing us to get the data from the AR Tag Toolbox.
