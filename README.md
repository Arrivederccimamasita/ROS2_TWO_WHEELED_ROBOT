## ROS2 Differential-drive Robot in Gazebo and Rviz

### Dolly Setup

[Reference](https://github.com/chapulina/dolly)

#### Install gazebo_ros_pkgs
$ sudo apt install ros-dashing-gazebo-ros-pkgs  

#### Build packages

        $ mkdir -p ~/ros2_simulation_ws/src  
        $ git clone https://github.com/chapulina/dolly  
        $ cd ~/ros2_simulation_ws  
        $ colcon build  

#### Setup environment variables

$ . /usr/share/gazebo/setup.sh  
$ . ~/ros2_simulation_ws/install/setup.bash  

#### Launch Dolly in a city 

$ ros2 launch dolly_gazebo dolly.launch.py world:=dolly_city.world  

#### Launch Dolly in an empty world

$ ros2 launch dolly_gazebo dolly.launch.py world:=dolly_empty.world  

### Packages

* `dolly`: Metapackage which provides all other packages.  
* `dolly_follow`: Provides node with follow logic.  
* `dolly_gazebo`: Robot model, simulation world and launch scripts.  