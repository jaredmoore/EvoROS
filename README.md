# EvoROS: Evolutionary Robotics with ROS and Gazebo

Authors:
Jared M. Moore, Grand Valley State University
Anthony J. Clark, Missouri State University
Glen A. Simon, Michigan State University
Philip K. McKinley, Michigan State University

## Software Used:

1. ROS [Robot Operating System](http://www.ros.org/)
2. Gazebo [Gazebo Simulation Environment](http://gazebosim.org/)
3. Ubuntu 14.04 (For ROS Indigo)

ROS has many versions, often tied to a specific installation of Ubuntu.  Depending on the packages used, you may need to adjust your installation versions accordingly.  In this repository, we are working with ROS-Indigo, Gazebo version 7, and Ubuntu 14.04.  This is currently due to requirements of additional robot control packages used in the ErleRover robotics suite.

## Installation Process:

While we detail the critical steps for installation below, up-to-date guides for the process are available at the following links:

* ROS: [http://wiki.ros.org/indigo/Installation/Ubuntu](http://wiki.ros.org/indigo/Installation/Ubuntu)
* Gazebo: [http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0)
* gazebo-ros-packages: [http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)

**Note:** Make sure to install the python-rosinstall package to use ROS calls within Python code!

1. Install ROS-Indigo - `sudo apt-get install ros-indigo-desktop`
2. Install Gazebo 7 - `sudo apt-get install gazebo7`
3. Install gazebo-ros-packages - `sudo apt-get install  ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros`
4. Install python-rosinstall - `sudo apt-get install python-rosinstall`
*Optional Packages*
5. Install smach - `sudo apt-get install ros-indigo-executive-smach` (Used for basicbot tutorials.)

**Note:** For different versions of ROS, you can typically replace `ros-indigo-...` with the new version, for example `ros-kinetic-...`

Other packages may be needed for your own experiments.  They generally follow the `sudo apt-get ros-indigo-` paradigm.  

## Basicbot Example:

The basicbot example provided in this package is intended to get you started with ROS and Gazebo.  For this simple task, the robot has two wheels and a LIDAR sensor.  Its vision is partitioned into three segments: left, center, and right.  The state machine controller, provided by the `smach` package, takes into account the sensor information and attempts to drive toward the cylinder placed in front of the robot.  

Fitness is based on a combination of distance from the cylinder and the time to reach a stopping threshold.  This is a minimization task.  

Currently, the example executes one master ROS server with four namespaced Gazebo simulations.  Each simulation runs independently of each other.  Depending on the available computing resources, the number of namespaced Gazebo instances can be increased or decreased.  Unfortunately, this is currently a manual process and is accomplished by editing the `basicbot_ga.launch` file.  

## Running the Basicbot Example:

In one terminal window, navigate to the base folder of the repository and run the following commands:

** Bootstrap the Packages **
1. `source /opt/ros/indigo/setup.sh`
2. `catkin_init`
3. `catkin_make`
4. `source /devel/setup.sh`

** Run the simulation. **
1. `roslaunch basicbot_ga basicbot_ga.launch`

Wait for the Gazebo instances to initialize, you will see a large amount of text written to your terminal.

Next, open another terminal window, navigate to the base folder of the repository and run the following commands:

1. `python src/basicbot_ga/test/ga_server.py --pop_size=4 --gens=2 --output_path="/home/<user>/"`

Set your output path according to your user folder, or where you want the output file to be written.  Increase population size and the number of generations as desired.  

** Current Limitations **
The simulations are not deterministic.  Therefore, methods to alleviate this are recommended.  Given this simple example, we have not implemented them here.

## Future Extensions:

1. Add additional examples.
2. Describe the setup of ROS/Gazebo architecture.