# open-dobot-gui
GUI Application to control a Dobot robotic arm via an Arduino Mega 2560 and a RAMPS 1.4 board. 

Inverse Kinematics Implemented! I will write up more about this some other time. There's a link in the code to a youtube video that explains most of it.

100% open source hardware and software

MIT License

The gui app is written in python and uses pyqt. It is not yet completely functional, but works enough for testing purposes (if you know how to code). Most of the remaining work on it is fairly easy to do, just takes time. Also included in this repository is a c++ qt project which performs a 3d simulation of the inverse kinematics. Usefulin verify that the inverse kinematics are correctly implemented. Currently, I'm having trouble defining the work area limits. I need to do that somehow. The lower arm angle limits is what's tripping me up. The inverse kinematics on the robot itself seems to work. Just be careful about transforming angles to the correct orientation. I might have messed that up a bit. If I did, it's subtle. 

![Alt text](/opendobotgui1.png?raw=true "Optional Title")

TO DO:

-Verify inverse kinematics and fix any problems

-Get angles from IMUs

-3D print IMU holder

-3D print endstop holder and connect end stops

-Implement emergency stop function in GUI

-much more, but too tired to list it all

