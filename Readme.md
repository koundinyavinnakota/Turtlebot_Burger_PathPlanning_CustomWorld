Implementation of A-star Algorithm using Turtlebot Burger



1.) cd catkin_ws/src ---> git clone the repository.
2.) Run the following commands:
--> cd ..
--> catkin_make
--> source devel/setup.bash
--> roslaunch astar astar.launch ( This will launch the turtlebot3 in the custom map inside the gazebo environement.)
--> Open a new terminal source the bash
--> rosrun astar mainNode.py


Giving Inputs to the program:
-> RPM1, RPM2 - (Advised rpms 1,0.5 or please give in range of 0 - 5 for optimum results)
-> Please enter the start point in range 0-100.( Please start from 3,3,0 as the turtlebot spawn position is fixed. In order to change the spawn location please navigate to the launch file and edit the first three lines corresponding to the robot pose and make the package)
-> Enter the goal location



After the path is displayed, please press 'q' on the image to start the turtlebot.


Dependencies:
-Python
-Gazebo
-ROS
-Opencv 4.1.0
-Numpy
-queue
