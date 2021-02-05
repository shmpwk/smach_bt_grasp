```
$ mkdir -p ~/YOUR_WORKSPACE/src
$ cd ~/YOU_WORKSPACE
$ catkin init
$ pip install scikit-robot[all] #https://github.com/iory/scikit-robot
$ sudo apt install ros-kinetic-pr2* #Install PR2 packages.
$ sudo apt install ros-kinetic-jsk-pr2*
$ cd ~/YOUR_WORKWPACE/src
$ git clone https://github.com/jsk-ros-pkg/jsk_recognition.git
$ cd ~/YOUR_WORKSPACE
#$ wstool init src
#$ wstool set jsk_demos https://github.com/jsk-ros-pkg/jsk_demos -t src --git
#$ wstool update -t src
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y -r #Install depedencies
$ catkin build jsk_pcl_ros
```
