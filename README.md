# 

**Abstract**
This projects aims for a ROS application that manages the autonomous motion planning of multiple UAVs. Hector Quadrotor package is being used for simulation. It is currently under development.

**Usage**

 - Copy the `spawn_multiple_quadrotors.launch` file into the `hector_quadrotor_gazebo` package.
 - Change the line in the launch file of `hector_quadrotor_demo` which includes the quadrotor that are to spawned, to include `spawn_multiple_quadrotors.launch`.

