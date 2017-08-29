# 

**Abstract**
This projects aims for a ROS application that manages the autonomous motion planning of multiple UAVs. Hector Quadrotor package is being used for simulation. It is currently under development, but most of the implementation has been done.

**Usage**

 - Go to [ Hector Quadrotor Repository](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor) and get a clone of it under `∼/catkin ws/src`
 - Get a close of this repository under `∼/catkin ws/src`
 - Go to `∼/catkin ws` and run `catkin_make`
 
 Open the terminal and run below commands sequentially.
 ```
  hg clone https://bitbucket.org/eigen/eigen/
  cd eigen
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
 ```
 
 - In `worlds/new.world` file, change the corresponding lines so that it directs to your specific root directory.(e.g home/burak, home/tahsin etc.)
 
 *Open two terminals. In each of them go to `∼/catkin ws/src` and run
`source devel/setup.bash`
This step is required to prepare the layer.*

After Eigen is succesfully installed and changes applied, all necessary installation is done. Now, in
 order to run the simulation two more commands are necessary:
 
 - In the first terminal run `roslaunch swarm_uav_manipulator simulate.launch`
 - In the second terminal run `roslaunch swarm_uav_manipulator quad manipulator.launch`
