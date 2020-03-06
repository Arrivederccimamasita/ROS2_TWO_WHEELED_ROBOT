## ROS2 Differential-drive Robot in Gazebo and Rviz

### Dolly Setup

[Reference1](https://github.com/chapulina/dolly)  
[Reference2](https://www.youtube.com/watch?v=Gwbk6Qf_TqY&t=1528s)  

#### Install gazebo_ros_pkgs
        $ sudo apt install ros-dashing-gazebo-ros-pkgs  

#### Build packages

        $ mkdir -p ~/ros2_simulation_ws/src  
        $ git clone 
        $ cd ~/ros2_simulation_ws  
        $ colcon build --symlink-install  

#### Setup environment variables


        $ source ~/ros2_simulation_ws/install/setup.zsh
<!-- $ source ~/ros2_simulation_ws/install/local_setup.bash -->



### Packages

* `dolly_planner`: Provides node with follow logic.

### Dolly Control

[Reference](https://www.youtube.com/watch?v=qB4SaP3TZog&t=994s)


### Run

<!-- $ cp -r ~/ros2_simulation_ws/src/dolly/dolly_gazebo/models/casual_female/ /usr/share/gazebo/models -->
        $ cd ~/ros2_simulation_ws
        $ colcon build --symlink-install  
        $ source /usr/share/gazebo/setup.sh  
        $ source install/setup.zsh 
        $ ros2 launch planner dolly_drive.launch.py  

<img src="./images/demo.gif" width="600" >
