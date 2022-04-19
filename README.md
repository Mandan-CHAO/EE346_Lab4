# lane following
This is the lab 4 for SUSTech EE346 Spring 2022.

It contains gazebo simulation model and control code for turtlebot3 burger.

# Usage

## 1. Clone the source code and catkin_make this package
  cd ~/catkin_ws/src
  
  git clone https://github.com/Mandan-CHAO/EE346_Lab4
  
  cd ..
  
  catkin_make

   
## 2. Launch the gazebo map
   export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/EE346_Lab4/models
   
   source ~/catkin_ws/devel/setup.bash
   
   roslaunch EE346_Lab4 race_track.launch 

## 3. chmod lane following python code
   cd ~/catkin_ws/src/EE346_Lab4/scripts/
   
   chmod +x lane_following_part1.py
   
   chmod +x lane_following_part2.py
   
   chmod +x lane_following_part3.py

   
## 4. run lane following control part
   cd ~/catkin_ws
   
   source devel/setup.bash
   
### part1
   rosrun EE346_Lab4 lane_following_part1.py
   
### part2
   rosrun EE346_Lab4 lane_following_part2.py
   
### part3
   rosrun EE346_Lab4 lane_following_part3.py
