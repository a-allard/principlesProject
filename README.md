# Unmaintained use if you choose at your own risk



# UR10 optimization based IVK

This repo contains source code for Allard's projects for 685 and 784 classes.
Initially this repo was setup for 685 as a simple UR10 robot picking up blocks and sorting by color.  During 784 the robot was revisited and
instead of using a prewritten URDF based IVK engine Allard wrote an optimizer based 


# Setup information
To use this repo you will need to setup Ubuntu 18.04 (a virtual machine is recommended but not required) with ROS melodic.  There should be no need to custom compile.  This should work out of the box.
All code is written to be python 2.7 compatible.

# Using
After you clone this do a `catkin_make_isolated`.  When that finishes (assuming no errors) run `roslaunch ur_gazebo ur10Gripper.launch`.  Finally in a new terminal run `rosrun allardControl ur10PickAndPlace.py`

# Future work
This project shows it is possbile to use an optimizer for IVK.  To truely explore this possibility the next step would be to rewrite this optimizer in C++ or Julia to have compiled languages rahter than python.  This should speed up the optimizer and make it much more competitive with KDL's IVK engine.

