LexyRobot
=========

Arduino Robotic Arm with 2 degrees of freedom controled by MATLAB. Project deverloped with Chris Zachariah as part of the course ECE 5995 Introduction to Robotics at Wayne State from [Dr Abhilash Pandya](http://ece.eng.wayne.edu/~apandya/). Documentation available at [Repository Wiki Page](https://github.com/akafael/LexyRobot/wiki)

## Folder Organization
 - [matlab](matlab)
   - [letters](matlab/letters): Generated handwriting letters .mat files
   - [pen](matlab/pen): Pen Script files
   - [lexyRobot.m](matlab/lexyRobot.mat) : Robot Class
   - [LexyRobotGUI.fig](matlab/LexyRobotGUI.fig) : Matlab GUI Figure file
   - [LexyRobotGUI.m](matlab/LexyRobotGUI.m) : Matlab GUI Script file
   - [testLexyRobot.m](matlab/testLexyRobot.m) : Matlab Script for unitary function testing

## LexyRobot GUI

![LexyRobotGUI](https://raw.githubusercontent.com/akafael/LexyRobot/master/doc/img/lexyRobotGUI.png)


### Features GUI

 - Scan COM ports and perform Connection
 - Inverse and Direct Kinematics
 - Control trough angle and coordinates in cm
 - Draw Words
 - Path Planning (DStar and PRM)

## Robot Image

![LexyRobot Hardware](https://github.com/akafael/LexyRobot/raw/master/doc/img/lexyRobotHardware.png)

## Reference
### Software
  - [Matlab Robotic Toolbox](http://petercorke.com/Robotics_Toolbox.html) Open Source Toolbox created by [@petercorke](https://github.com/petercorke)
  - [ArduinoIO](https://www.mathworks.com/hardware-support/arduino-matlab.html?requestedDomain=www.mathworks.com) Arduino Legacy Support for Matlab
  - [Object Oriented Programing on Matlab](https://www.mathworks.com/company/newsletters/articles/introduction-to-object-oriented-programming-in-matlab.html) Concepts used in the project
  - [Pen](https://www.mathworks.com/matlabcentral/fileexchange/26225-pen---mouse-draw-input-template) Pen / Mouse draw input template modified for this project

### Related Videos
  - [Robotic Theory](https://www.youtube.com/watch?v=0yD3uBshJB0&list=PL65CC0384A1798ADF)
  - [Project Results](https://www.youtube.com/watch?v=L9-bt8S9pyk&list=PLLVv9YVxXD97kZRXK5z1hEUbqRAv6iZf2)
