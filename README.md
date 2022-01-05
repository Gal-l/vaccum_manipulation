# vaccum_manipulation_ws

installation: abunto 18.04
1. ros melodic full desktop
2.  install ros indastial - abb package: https://github.com/ros-industrial/abb_experimental
# change to the root of the Catkin workspace
$ cd $HOME/catkin_ws

# retrieve the latest development version of the abb repository. If you'd rather
# use the latest released version, replace 'kinetic-devel' with 'kinetic'
$ git clone -b kinetic-devel https://github.com/ros-industrial/abb.git src/abb
# retrieve the latest development version of abb_experimental
$ git clone -b kinetic-devel https://github.com/ros-industrial/abb_experimental.git src/abb_experimental

# check build dependencies. Note: this may install additional packages,
# depending on the software installed on the machine
$ rosdep update

# be sure to change 'kinetic' to whichever ROS release you are using
$ rosdep install --from-paths src/ --ignore-src --rosdistro kinetic

# build the workspace (using catkin_tools)
$ catkin build
3. install moveit 
http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html#install-moveit
4. sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
http://wiki.ros.org/ros_control

