# HAND-TO-EYE CALIBRATION WITH THE DOBOT MAGICIAN


## By Connor Eyles, Joshua Poot and Travis Goodshaw


### Project Outline
The code included in this repository is for calibrating the position of the objects and robot base with respect to the camera. 
The hand to eye calibration is done by finding the positions of the AR Tags using the ar_tag_toolbox and an RGB-D Camera.
The AR Tag for the base is transformed by multiplying the transform offset and inverting this to turn it from the camera frame into the robot frame of reference.
The GUI allows us to connect, disconnect and initialise to the Dobot. It offers a user the functionality to calibrate the tag positions with respect to the camera, move the dobot to each tag position and switch to tag follow mode. The GUI also displays the X,Y and Z position of the tags with respect to the camera. 
If the tags or the robot have been moved, the GUI also allows for recalibration without having to reinitialise the entire program.
If required, the transforms of all the Camera, the AR Tags, the Robot Base and the Robot End Effector can be plotted in a graph.

The program has two primary functions:

- Moving to Tags: The Dobot will move to any of the calibrated AR Tags in visual range. The GUI allows the user to select which tag it can move to, but will not move to tags that aren't in range.

- Follow Mode: The Dobot will follow a single AR Tag, even if the AR Tag is moved but still in range of the sensor.
