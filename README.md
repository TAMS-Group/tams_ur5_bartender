# Bartender Project 2016/2017

This is the result of a two term master project at the [Technical Aspects of Multimodal Systems (TAMS) Group](https://tams-www.informatik.uni-hamburg.de/) at the [University of Hamburg](https://www.uni-hamburg.de/).


## Objective
The goal of the project is to let an [UR5 robot arm](https://www.universal-robots.com/products/ur5-robot/) with [Robotiq 3-Finger Adaptive Robot Gripper](http://robotiq.com/products/industrial-robot-hand/) perform a complex grasping and pouring manipulation based on visual data.
In other words, the robot arm is supposed prepare a drink following a cocktail recipe and based on user input from a web interface.


## Getting Started

The fastest way of getting started is to clone the repository that subsumes all required repos as submodules; with the  *recursive* flag set the submodules are directly cloned as well:

```
git clone --recursive ssh://gitolite@git.mafiasi.de/masterproject-intelligent-robotics/2016-masterproject-intelligent-robotics/bartender-project.git
```

However, if the host repository has already been cloned the "usual" way, the same thing can be achieved by explicitly initializing and updating the submodules:

```
cd bartender-project
```

```
git submodule update --init
```

**Relevant for devel:**

**The host repository, in our case "bartender-project", will point towards particular versions of the submodules and thus not automatically towards the newest commit on the master branch. Therefore, cloning or updating the submodules as described above might result in older commits rather than the newest commits on branches.**

The [*remote* flag](https://git-scm.com/docs/git-submodule#git-submodule---remote) takes care of that:

```
git submodule update --init --remote
```

If the HEAD of a submodule is detached which seems to happen quite often in the world of submodules, one can fix that with an ```git checkout master``` for that submodule or just do it for all via:

```
git submodule foreach git checkout master
```

An alternativ to the ```git submodule update --init --remote``` command and maybe easier to remember is the following:

```
git submodule foreach git pull origin master
```



## Dependencies

* ```project16_manipulation```
   * [MoveIt!](http://moveit.ros.org/)
     * we encountered a couple of bugs which we bypassed in the original code
   * [```tams_ur5_setup```](https://github.com/TAMS-Group/tams_ur5_setup.git) with branch 'project2016'
   * [```tams_ur5```](https://github.com/TAMS-Group/tams_ur5.git)
* ```project16_image_processing```
   * [```OpenCV```](http://opencv.org/)
   * [```Kinect driver```](http://wiki.ros.org/freenect_launch)
   * [```apriltags_ros```](http://wiki.ros.org/apriltags_ros)
   * [```Camera Positioner```](https://github.com/TAMS-Group/camera_positioner)
* ```project16_gui```
	* [```rosbridge_suite```](http://wiki.ros.org/rosbridge_suite)



## Running The Bartender

After getting the host repo and all the dependencies do a ```catkin_make``` and off you might go:


### Run On Hardware

```
roslaunch project16_coordinator project16.launch
```


### Run In Simulation

Simulate everything in RVIZ: the image processing part is completely omitted and covered by ```publishDemoBottles.launch```:

```
roslaunch project16_coordinator simulationProject16.launch 
```





## Submodules

* ```find_object_2d```

* ```kinect_calibration```

* ```pr2016_msgs```

* ```project16_coordinator```

* ```project16_gui```

* ```project16_image_processing```

* ```project16_manipulation```




