# ENPM 661 Project 3 Phase 2

## A* ALGORITHM on Turtlebot

### Student Names
- Shantanu Parab
- Vineet Singh

### Directory ID & UID
- sparab - 119347539
- vsingh03 - 119123614

## Libraries used are: 
>numpy, timeit, queue, OpenCV, math 

<!-- ## Source Code Files

- [`phase2map1.py`](phase2map1.py) - Contains the implementation of Astar algorithm to solve the path planning problem.
  + The program will run a A* algorithm for path planning and create a video. This video will be saved after execution as Astar01.avi.
  + Once the program runs  completetly it will show  a image output of optimum path.
  + Press enter once the image is shown this will complete the video recording
  + The program will ask the user to give the inputs for start and goal and check wheter it is valid.
  + Selecting Robot Clearance values and Object Clearance values above 5 will make the right most area on the map inaccessible. 

- [`Astar_turtlebot.py`](Astar-turtlebot.py) - Contains the implementation of Astar algorithm to solve the path planning problem.
  + The program will run a A* algorithm for path planning and create a video of the entire node generation. This video will be saved after execution as Astar02.avi.
  + Once the program runs  completetly it will show  a image output of optimum path.
  + Press enter once the image is shown this will complete the video recording
  + The program will ask the user to give the inputs for start and goal and check wheter it is valid.


Note: The output videos in both cases is created as .avi which is converted to .mp4 using another software. -->

  <!-- To test any other initial state and goal state, provide the states to --InitState and --GoalState parameter while running the code (x y theta). 

  The canvas size is 600\*200 for the map in Astar_turtlebot.py
  The canvas size is 600\*250 for the map in phase2map1.py
  The inputs of initial and goal nodes should be given in Cartetsian Coordinates, and Step size between 1 to 10.  -->

## How to Run the Program:

To run the program, clone the package in catkin workspace and then follow the below steps:
+ Open the terminal and move to catkin workspace where package is cloned. Move to root of the workspace. 
+ Build the Catkin workspace 
+ Source the workspace after build is complete
+ Run the following command 

      roslaunch astar_turtlebot gazebo.launch 


+ Enter the values for each prompt (tested values given in Note below).  
+ The program will output the final backtracked path as OpenCV prompt window. Select the OpenCV prompt window and **Press Enter**.
+ Turtlebot will start following the path in gazebo and values will be dispalyed on the terminal. 
  


### Note: 
+ The speeds for Left and right wheels should be given after consideration of turtlebot max limit for speed/acceleration. For proper outputs, keep it within 20 rpm for each wheel, and difference between both rpms to be within 10 for optimal performance. 
+ The inputs and goal points should be given with reference to ROS map. If initial state is given other than (0 0 0), then it should also be changed in Launch file before starting the node to correctly spawn the robot. 
+ As the controller is open loop, the robot might not follow the exact path as per the final action set. So we suggest using the same inputs as given below, as this is already tested. 
  + Enter robot clearance: 20
  + Enter object clearance: 5
  + Enter robot left rpm: 15
  + Enter robot right rpm: 10
  + Enter Start X: 0
  + Enter Start Y: 0
  + Enter Start Theta: 0
  + Enter Goal X: 4.8
  + Enter Goal Y: -0.5
  + Enter Goal Theta: 0
+ Robot Clearance should be at least 20 for optimal performance. 
+ The program will first execute A* and generate set of actions to reach the goal which will be stored as actions.txt. 
+ Turtlebot controller node reads the actions.txt and drives the robot as per the actions to reach the final destination.


## Output Video: 
https://youtu.be/9bkgCtHp0WE -  Shows the complete process

https://youtu.be/8U9iF_QIr0E -  Shows comparison with backtracked path


## Github Link
[Repository](https://github.com/VKSingh03/astar_turtlebot_ros_noetic.git)

<!-- ## Video Output
[Output Video for map1](Astar01.mp4) -->




