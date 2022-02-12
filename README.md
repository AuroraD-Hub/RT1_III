# Gazebo & Rviz simulation
This third assignment consists in a robot simulation based on ROS environment and in c++ language. Gazibo and Rviz environment has been downloaded from [Prof. Carminee Recchiuto's repository](https://github.com/CarmineD8/final_assignment) for Robotics Engineering students of UNIGE.

Some parameters has been already set in [slam_gmapping package](https://github.com/CarmineD8/slam_gmapping) needed for this simulation.

## Installing and running
At first, install the above packages in your ROS workspace by executing in terminal:
```
git clone https://github.com/CarmineD8/final_assignment
git clone https://github.com/CarmineD8/slam_gmapping
```
Since ROS noetic is used, make sure to also have the 'ros navigation stack' installed by executing:
```
apt-get install ros-noetic-navigation
```
Switch on the noetic branch of the packeges by executing:
```
git checkout noetic
```
At last, download this package and build the workspace by executing `catkin_make` command.

Now, to run the simulation digit respectively in three different terminals:
```
roslaunch final_assignment simulation_gmapping
roslaunch final_assignment move_base
rosrun final_assignment interface_node
```
Gazebo and Rviz environment will be available.

## Assignment
