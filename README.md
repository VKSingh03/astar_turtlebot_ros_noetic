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

    roslaunch astar_turtlebot gazebo.launch init_state:="0 0 0" goal_state:="4.8 -0.5 0" rpm1:=10 rpm2:=15 obj_clear:=5 robot_clear:=20

Output Video: https://youtu.be/8U9iF_QIr0E


Note: 
+ The speeds for Left and right wheels should be given after consideration of turtlebot max limit for speed/acceleration. For proper outputs, keep it within 20 rpm for each wheel. 
+ The inputs and goal points should be given with reference to ROS map. If initial state is given other than (0 0 0), then it should also be changed in Launch file before starting the node. 
+ As the package already contains an actions.txt, the turtlebot will not wait for the new file to be generated. If a new action set is to be generated, the existing actions.txt should be deleted and then the above launch file should be run again. 

## Github Link
[Repository](https://github.com/VKSingh03/astar_turtlebot_ros_noetic.git)

<!-- ## Video Output
[Output Video for map1](Astar01.mp4) -->




