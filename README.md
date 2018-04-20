# Pepper Protocol


## Getting Started

We are using the most recent version of ROS that support the connexion of Pepper, so it need to be install and configure on your computer before we start.

### Prerequisites

Require : 
* [Ubuntu](https://ubuntu-fr.org/telechargement) - 16.04
* [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu) - Version Kinetic

Create a workspace in your path.

### Installing

Base package of navigation, viewing camera are needed.

In a terminal execute : 

```
sudo apt-get ros-kinetic-move-base-msgs ros-kinetic-octomap ros-kinetic-octomap-msgs ros-kinetic-humanoid-msgs ros-kinetic-humanoid-nav-msgs ros-kinetic-camera-info-manager ros-kinetic-camera-info-manager-py 
```

Next comes the package report to Pepper.

```
sudo apt-get install ros-kinetic-pepper-.*
```

A drive can be found on github, so clone it in your workspace like so:

```
git clone https://github.com/ros-naoqi/naoqi_driver.git
```

Do a *catkin_make* to build.

Now ros have all the package to control Pepper, but it miss the sdk.

You can found it on the [SoftBank Robotics web site.](https://developer.softbankrobotics.com/us-en/downloads/pepper)

Download pepper linux64 and python2.7 sdk.

## Running the package

### PYTHONPATH

You need to inform the path of your naoqi sdk in the environnement variable PYTHONPATH.

```
export PYTHONPATH=$PYTHONPATH:yourUserPath/sdkDirectory/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages
```

To avoid doing that in every terminal, modify your *.bashrc*.

### Connection to Pepper

Start pepper, wait until it move.

Start *roscore*. And now you need to know the ip adress of Pepper by pressing the chest button.

The computer ip and the interface can be read with

```
ifconfig -a
```

Complete the folowing instruction with your adress.

```
* Wifi or Access Point : 
roslaunch pepper_bringup pepper_full.launch nao_ip:=<ip of pepper> roscore_ip:=<pc ip> network_interface:=wlan0

* Ethernet or local :
roslaunch pepper_bringup pepper_full.launch nao_ip:=<ip of pepper> roscore_ip:=localhost network_interface:=eth0
```
## Control Pepper

Now ros in connected to pepper, you can illustrate it by publishing on the */cmd-vel* topic like so

```
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

And Pepper moves forward =) .

For more movement precision use the topic */move_base_simple/goal* by publishing a goal

```
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```
Don’t forget the *w orientation* or Pepper going to be **out of control**.

## See through Pepper

For that you first need to clone a package in your workspace.

```
https://github.com/ros-perception/image_pipeline.git
```

Build with *catkin_make* and run it by : 

```
rosrun image_view image_view image:=/pepper_robot/naoqi_driveramera/front/image_raw
```

And a windows appear with the front camera of pepper.
