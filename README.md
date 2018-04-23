# Bartender Project 2016/2017

This is the result of a two term master project at the [Technical Aspects of Multimodal Systems (TAMS) Group](https://tams-www.informatik.uni-hamburg.de/) at the [University of Hamburg](https://www.uni-hamburg.de/).


## Objective
The goal of the project is to let an [UR5 robot arm](https://www.universal-robots.com/products/ur5-robot/) with [Robotiq 3-Finger Adaptive Robot Gripper](http://robotiq.com/products/industrial-robot-hand/) perform a complex grasping and pouring manipulation based on visual data.
In other words, the robot arm is supposed prepare a drink following a cocktail recipe and based on user input from a web interface.



## Running The Bartender

After getting the host repo and all the dependencies do a ```catkin_make``` and off you might go:
The demo can be run both with and without hardware.
Each method only requires launching a single launch file, which will start all necessary processes,
drivers and a simple web interface from which the cocktails can be ordered.


### Run On Hardware

```
roslaunch tams_ur5_bartender bartender.launch
```

### Run In Simulation

The demo can be simulated and visualized in RVIZ completely by running
```
roslaunch tams_ur5_bartender simulation.launch 
```


### Packages



* ```tams_ur5_bartender_msgs```

* ```tams_ur5_bartender_coordination```

* ```tams_ur5_bartender_gui```

* ```tams_ur5_bartender_image_processing```

* ```tams_ur5_bartender_manipulation```

## Dependencies

* ```tams_ur5_bartender_manipulation```
   * [MoveIt!](http://moveit.ros.org/)
     * we encountered a couple of bugs which we bypassed in the original code
   * [```tams_ur5_setup```](https://github.com/TAMS-Group/tams_ur5_setup.git) with branch 'project2016'
   * [```tams_ur5```](https://github.com/TAMS-Group/tams_ur5.git)
* ```tams_ur5_bartender_image_processing```
   * [```OpenCV```](http://opencv.org/)
   * [```Kinect driver```](http://wiki.ros.org/freenect_launch)
   * [```apriltags_ros```](http://wiki.ros.org/apriltags_ros)
   * [```Camera Positioner```](https://github.com/TAMS-Group/camera_positioner)
* ```tams_ur5_bartender_gui```
	* [```rosbridge_suite```](http://wiki.ros.org/rosbridge_suite)






