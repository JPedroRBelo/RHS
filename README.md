# RHS - Robot House Simulator

The Robot House Simulator (RHS) is a simulator for Social Robotics. The simulator is part of a larger system, called CMDE, described in the OntSense repository at: github.com/helioaz/ontSense/. RHS is developed using the Unity Engine, under GPL v3.0 license.

## ROS and RHS

This repository provides a Robot Operating System (ROS) package for communicating and sending commands to the RHS. The package was written in C ++ using the Kinetic Kame version of ROS. (under construction)
## How to Run
This version of RHS supports Windows only. The Unity version utilized is 2017.2.

The RHS simulator will work perfectly without a cognitive architecture (or other decision-making system) and without a triplestore database (github.com/helioaz/ontSense/). You can view the sensing data through the lower left panel and send commands to the robot via ROS or via socket messages. 

To run RHS with ROS, you need to start running RHS on a Windows machine and the rhs_ros_bridge node on a Linux machine, both on the same network. The rhs_ros_package package contains a simple program (rhs_ros_talker.cpp) that allows sending commands to the rhs_ros_bridge.cpp, which is responsible for sending the message to the RHS via socket.

-modify the rhs_ros_bridge.cpp file and change the IP of the RHS host machine;
-compile the package rhs_ros_package with catkin_make
-make sure the RHS is running;
-run roscore;
-run rhs_ros_package rhs_ros_bridge;
-run rhs_ros_package rhs_ros_talker.


## More information

Visit https://github.com/JPedroRBelo/RHS/wiki for more informations about RHS


## Publications

- J. P. R. Belo, R. A. F. Romero and H. Azevedo, "RHS simulator for robotic cognitive systems", 2017 Latin American Robotics Symposium (LARS) and 2017 Brazilian Symposium on Robotics (SBR), Curitiba, 2017, pp. 1-6.
doi: 10.1109/SBR-LARS-R.2017.8215306, URL: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8215306&isnumber=8215260

- J. P. R. Belo, R. A. F. Romero and H. Azevedo, "SIMULADOR PARA SISTEMAS COGNITIVOS VOLTADO PARA ROBOTICA SOCIAL", 2017 XIII Simpósio Brasileiro de Automação Inteligente (SBAI2017), Porto Alegre, Brazil, 2017, pp. 51-56, URL: https://www.ufrgs.br/sbai17/papers/paper_23.pdf

- H. Azevedo, J. P. R. Belo and R. A. F. Romero, "Cognitive and robotic systems: Speeding up integration and results," 2017 Latin American Robotics Symposium (LARS) and 2017 Brazilian Symposium on Robotics (SBR), Curitiba, 2017, pp. 1-6.
doi: 10.1109/SBR-LARS-R.2017.8215337, URL: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8215337&isnumber=8215260

- H. Azevedo, R. A. F. Romero and J. P. R. Belo, "Reducing the gap between cognitive and robotic systems," 2017 26th IEEE International Symposium on Robot and Human Interactive Communication (RO-MAN), Lisbon, 2017, pp. 1049-1054.
doi: 10.1109/ROMAN.2017.8172433
URL: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8172433&isnumber=8172268



