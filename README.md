# MRLVR 2021

Here is MRL VR team source code from Qazvin Islamic Azad University. 

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them

```
- Ubuntu 18.04.3 LTS
- ROS Melodic Desktop Full
- QTCreator 5.15

```

### Install the ROS Melodic

A step by step series of packages and softwares that you need to install them.

Install ROS Melodic, Gazebo and QT Creator
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt install ros-melodic-desktop-full

sudo apt-get install git  ros-melodic-costmap-2d ros-melodic-nav-core ros-melodic-base-local-planner ros-melodic-navfn ros-melodic-move-base

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

source ~/.bashrc

sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

sudo apt install python-rosdep

sudo rosdep init

rosdep update

```

Clone & build  MRLVR 2021 project from gitlab repository:

```
git clone https://gitlab.com/mrlvr/vr2021.git

cd ~/vr2019

catkin_make

echo "source /home/[your username]/vr2021/devel" >> ~/.bashrc

source ~/.bashrc
```
Clone and & build RoboCup2021 Rescue Simulation Virtual Robot League Maps:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'   
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116   
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'   
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -     
sudo apt-get update   
sudo apt-get install -y cmake g++ protobuf-compiler pavucontrol gazebo9-plugin-base libgazebo9 libgazebo9-dev ros-melodic-desktop  ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-image-view2 ros-melodic-rqt ros-melodic-rqt-common-plugins ros-melodic-joy ros-melodic-teleop-twist-keyboard ros-melodic-message-to-tf ros-melodic-tf2-geometry-msgs ros-melodic-audio-common ros-melodic-costmap-2d ros-melodic-image-transport ros-melodic-image-transport-plugins ros-melodic-hector-mapping ros-melodic-hector-geotiff 
sudo apt-get install ros-melodic-hector-sensors-description ros-melodic-controller-manager ros-melodic-gmapping ros-melodic-move-base ros-melodic-hector-mapping ros-melodic-gazebo9*


git clone https://github.com/RoboCupRescueVirtualRobotLeague/RoboCup2021RVRL_Demo  
cd ~/RoboCup2021RVRL_Demo/src  
rosinstall src /opt/ros/melodic  
catkin_make

```

## Running the server
To run maps and spawn robots series of commands is need to run:

- Create bring_up.sh file in your home direcotory:
```
gedit bring_up.sh
```
- Copy these commands in file:
```
#!/bin/bash

world_launch="world_final_indoor_1.launch"
spawn_launch="spawn_multi_robots_final_indoor_1.launch"
vrpath="/home/mrlvr/Robocup2018RVRL"

tab1="bash -c '. setup.bash';'roslaunch setup $world_launch';bash"
tab2="bash -c '. setup.bash';'sleep 5';'roslaunch robot_description $spawn_launch';bash"
gnome-terminal --tab --working-directory="$vrpath" -e "$tab1" --tab --working-directory="$vrpath" -e "$tab2"
exit 0
```
- Save file and close it

- Run these command to set executable attribute on file:
```
chmod +x bring_up.sh
```

- Open Terminal :
```
./bring_up.sh
```


## Running the MRLVR 2021 GUI
Run bellow command to start mrl_rqt_dashboard:
```
rosrun mrl_dashboard start.py
```



## Authors


* **Mohammad H. Shayesteh** - *Software Developer* - (m.h.shayesteh@gmail.com)


* **Mohammad M. Raeisi** - *Software Developer* - (mahdrsi@gmail.com)

## License

This project is licensed under the Mechatronics Research Laboratory.
