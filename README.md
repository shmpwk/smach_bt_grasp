# Grasp Demo


## Initial Setup

```
$ mkdir -p ~/YOUR_WORKSPACE/src
$ cd ~/YOUR_WORKSPACE
$ catkin init
$ pip install scikit-robot[all] #https://github.com/iory/scikit-robot
$ sudo apt install ros-kinetic-pr2* #Install PR2 packages.
$ sudo apt install ros-kinetic-jsk-pr2*
$ cd ~/YOUR_WORKWPACE/src
$ git clone https://github.com/jsk-ros-pkg/jsk_recognition.git
$ cd ~/YOUR_WORKSPACE
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y -r #Install depedencies
$ catkin build jsk_pcl_ros
```

Maybe more library should be installed...



## smach example (grasp a can)
Based on https://github.com/jsk-ros-pkg/jsk_roseus/tree/master/roseus_smach
```
$ roslaunch grasp_demo common.launch
$ rosrun grasp_demo state-machine-ros.l #After that command `(exec-smach)`
$ rosrun smach_viewer smach_viewer.py 
```


 
## behavior tree example (grasp a can)
Based on https://github.com/miccol/ROS-Behavior-Tree
First, you should build behavior_tree_core package.
On each terminal and after `source ~/YOUR_WORKSPACE/devel/setup.bash`,
```
$ roslaunch grasp_demo common.launch
$ rosrun grasp_demo reactive_server.l
$ rosrun behavior_tree_core reactive_grasp 
```

Then behavior Tree show up, and the robot automatically grasps can.





## contact
For more infomation, please contact me.
wakabayashi-shumpei@g.ecc.u-tokyo.ac.jp


If you want to try python robot instead of euslisp, try [scikit robot](https://github.com/iory/scikit-robot)!
