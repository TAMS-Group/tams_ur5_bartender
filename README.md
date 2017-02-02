
# Bartender Project 2017
This is the result of a two term master project at the [Technical Aspects of Multimodal Systems (TAMS) Group](https://tams-www.informatik.uni-hamburg.de/) at the [University of Hamburg](https://www.uni-hamburg.de/).



## Objective
The goal of the project is to let an [UR5 robot arm](https://www.universal-robots.com/products/ur5-robot/) with [Robotiq 3-Finger Adaptive Robot Gripper](http://robotiq.com/products/industrial-robot-hand/) perform a complex grasping and pouring manipulation based on visual data. 


## Getting Started

The fastest way of getting started is to clone the repository that subsumes all required repos as submodules; with the  *recursive* flag set the submodules are directly cloned as well:

```
git clone --recursive ssh://gitolite@git.mafiasi.de/masterproject-intelligent-robotics/2016-masterproject-intelligent-robotics/bartender-project.git
```

However, if the host repository has already been cloned the "usual" way, the same thing can be achieved by explicitly initializing and updating the submodules:
```
cd bartender-project
git submodule update --init
```

### Relevant for devel:
**The host repository, i.e. in our case **bartender-project**, will point towards particular versions of the submodules and thus not automatically towards the newest commit on the master branch. Therefore, cloning or updating the submodules as described above might result in older commits rather than the newest commits on branches.**

The [*remote* flag](https://git-scm.com/docs/git-submodule#git-submodule---remote) takes care of that:
```
git submodule update --init --remote
```
If the HEAD of a submodule is detached which seems to happen quite often in the world of submodules, one can fix that with an ```git checkout master``` for that submodule or just do it for all via:
```
git submodule foreach git checkout master
```
Afterwards this might do the trick for pulling from the master:
```
git submodule foreach git pull origin master
```


## Dependencies

[MoveIt!](http://moveit.ros.org/)



## Submodules
#### "find_object_2d

#### "kinect_calibration"

#### "pr2016_msgs"

#### "project16_coordinator"

#### "project16_gui"

#### "project16_image_processing"

#### "project16_manipulation"




