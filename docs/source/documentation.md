@mainpage Overview

This is the documentation for the Bebop Controller Package. You can download it from its
<a href="https://github.com/Francisco8382/bebop_controller"><b>GitHub</b></a>. More detailed installation instructions can be found at this @link installation page@endlink.

Bebop Controller is a ROS Package that allows controlling the Parrot Bebop 2 drone in a real environment using an OptiTrack Motion Capture System to obtain the position data of the drone. It also includes a node that allows using Sphinx Simulator and obtain the position data of the drone from the simulator. 

To see the list of ROS nodes included, you can go to this @link ros_nodes page@endlink.

To run multiple nodes at the same time, it is recommended to use the * roslaunch * command. This requires a launch file, for which some are included in the *launch* folder. On this @link launch_files page@endlink you can see the included launch files.

ROS nodes require some parameters, which can be specified directly in launch files or with YAML files. The advantage of doing it with YAML files is that when modifying a parameter that is used in several controllers, such as those of the reference trajectory, they are only modified in a single file and this affects all controllers. On this @link yaml_files page@endlink you can see the included YAML files.

@page installation Installation Instructions
[TOC]

@section prerequisites Prerequesite Installation

@subsection bebopautonomy Package 'bebop_autonomy'

In a bash terminal, the following commands are run to install the package.

~~~{.bash}
mkdir -p ~/bebop_ws/src
cd ~/bebop_ws
catkin init
cd ~/bebop_ws/src
git clone https://github.com/AutonomyLab/bebop_autonomy
cd ~/bebop_ws
catkin build
echo source ~/bebop_ws/devel/setup.bash >> ~/.bashrc
~~~

@subsection vrpninstallation Package 'vrpn_client_ros'

This will be necessary in case you want to test with the real drone and the Optitrack Motion Capture System.

To install the package, the following command is run in a bash terminal.

~~~{.bash}
sudo apt-get install ros-kinetic-vrpn-client-ros
~~~

@subsection sphinxinstallation Sphinx Simulator

This will be necessary in case you want to test the controllers in Gazebo.

The following commands are run to install Sphinx Simulator.

~~~{.bash}
echo "deb http://plf.parrot.com/sphinx/binary `lsb_release -cs`/" | sudo tee /etc/apt/sources.list.d/sphinx.list > /dev/null
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 508B1AE5
sudo apt-get update
sudo apt-get install parrot-sphinx
~~~

Once installed, you must modify a file, for which the following command is run.

~~~{.bash}
sudo gedit /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone
~~~

The content of the file is replaced with the following.

~~~{.bash}
<?xml version="1.0" encoding="UTF-8"?>
	<drone
		name="bebop2"
		firmware="http://plf.parrot.com/sphinx/firmwares/ardrone3/milos_pc/latest/images/ardrone3-milos_pc.ext2.zip"
		hardware="milosboard">
	<machine_params
		low_gpu="1"
		with_front_cam="0"
		with_hd_battery="0"
		with_flir="0"
		flir_pos="tilted"/>
	<pose>default</pose>
	<interface>eth1</interface>
</drone>
~~~

@section pkg_installation Package Installation

The GitHub of the package can be found at this <a href="https://github.com/Francisco8382/bebop_controller"><b>link</b></a>.

In a bash terminal, the following commands are run to create a workspace in which to install the package.

~~~{.bash}
mkdir -p ~/bebop_controller/src
cd ~/bebop_controller
catkin init
~~~

Files are cloned from the GitHub using the following commands. 

~~~{.bash}
cd ~/bebop_controller/src
git clone https://github.com/Francisco8382/bebop_controller
git clone https://github.com/ethz-asl/mav_comm
~~~

Finally, they are compiled by running the following.

~~~{.bash}
cd ~/bebop_controller
catkin build
echo source ~/bebop_controller/devel/setup.bash >> ~/.bashrc
~~~

@page ros_nodes ROS Nodes
[TOC]

The *src/nodes* and *scripts* folders contain the ROS nodes. 

We can find the following node files in these folders.

- citc_controller_angles.cpp
- citc_controller_twist.cpp
- data_to_csv.cpp
- gazebo.py
- pid_controller_angles.cpp
- pid_controller_twist.cpp
- plot.py
- proportional_controller.cpp
- sinusoidal.cpp
- square_root_controller.cpp

To run these nodes it is recommended to create a launch file and run them with the *roslaunch* command. It can also be run using the *rosrun* command, but it is more difficult to specify the parameters.

Some @link launch_files Launch Files@endlink are included in the *launch* folder. 

@page launch_files Launch Files
[TOC]

The *launch* folder contains files that are used to run one or more ROS nodes.

In this folder we can find the following files.

- citc_controller_angles.launch
- citc_controller_twist.launch
- gazebo.launch
- pid_controller_angles.launch
- pid_controller_twist.launch
- proportional_controller.launch
- square_root_controller.launch
- vrpn_client_ros.launch

Some parameters can be modified directly from these files, but most parameters are modified from YAML files. These parameters are explained in this @link yaml_files section@endlink.

To run these files, the following command is run in the terminal.

~~~{.bash}
roslaunch bebop_controller <name_of_the_file>
~~~

For example, to run the *vrpn_client_ros.launch* file.
~~~{.bash}
roslaunch bebop_controller vrpn_client_ros.launch
~~~

@file citc_controller_angles.launch
@brief Launch file to run a test using the CITC controller with reference angles.

This launch file runs the following nodes.
- citc_controller_angles.cpp
- sinusoidal.cpp
- data_to_csv.cpp
- plot.py
- gazebo.py


@file citc_controller_twist.launch
@brief Launch file to run a test using the CITC controller with velocity commands.

This launch file runs the following nodes.
- citc_controller_twist.cpp
- sinusoidal.cpp
- data_to_csv.cpp
- plot.py
- gazebo.py

@file gazebo.launch
@brief Launch file example to run the publisher that gets the position data from the simulator.

This launch file runs the following node.
- gazebo.py

@file pid_controller_angles.launch
@brief Launch file to run a test using the PID controller with reference angles.

This launch file runs the following nodes.
- pid_controller_angles.cpp
- sinusoidal.cpp
- data_to_csv.cpp
- plot.py
- gazebo.py

@file pid_controller_twist.launch
@brief Launch file to run a test using the PID controller with velocity commands.

This launch file runs the following nodes.
- pid_controller_twist.cpp
- sinusoidal.cpp
- data_to_csv.cpp
- plot.py
- gazebo.py

@file proportional_controller.launch
@brief Launch file to run a test using the proportional controller.

This launch file runs the following nodes.
- proportional_controller.cpp
- sinusoidal.cpp
- data_to_csv.cpp
- plot.py
- gazebo.py

@file square_root_controller.launch
@brief Launch file to run a test using the square root controller.

This launch file runs the following nodes.
- square_root_controller.cpp
- sinusoidal.cpp
- data_to_csv.cpp
- plot.py
- gazebo.py

@file vrpn_client_ros.launch
@brief Launch file example to use vrpn client as ROS publisher.

This launch file runs the following node.
- vrpn_client_ros

@page yaml_files YAML Files
[TOC]

YAML files are used to set the parameters used by ROS nodes. These files are in the *resource* folder.

The following files are located in this folder. 

- citc_controller_angles.yaml
- citc_controller_twist.yaml
- max_speed.yaml
- normalize_angles.yaml
- normalize_twist.yaml
- pid_controller_angles.yaml
- pid_controller_twist.yaml
- proportional_controller.yaml
- safe_zone.yaml
- square_root_controller.yaml
- topics.yaml
- trajectory.yaml
- waypoint.yaml

@file citc_controller_angles.yaml
@brief YAML file for CITC controller parameters, using reference angles.

The following parameters can be configured in this file.
@param Gains/K1x Value of @f${k_1}_x@f$
@param Gains/K1y Value of @f${k_1}_y@f$
@param Gains/K1z Value of @f${k_1}_z@f$
@param Gains/K1yaw Value of @f${k_1}_\psi@f$
@param Gains/K2x Value of @f${k_2}_x@f$
@param Gains/K2y Value of @f${k_2}_y@f$
@param Gains/K2z Value of @f${k_2}_z@f$
@param Gains/K2yaw Value of @f${k_2}_\psi@f$
@param Gains/K3x Value of @f${k_3}_x@f$
@param Gains/K3y Value of @f${k_3}_y@f$
@param Gains/K3z Value of @f${k_3}_z@f$
@param Gains/K3yaw Value of @f${k_3}_\psi@f$
@param Gains/K4x Value of @f${k_4}_x@f$
@param Gains/K4y Value of @f${k_4}_y@f$
@param Gains/K4z Value of @f${k_4}_z@f$
@param Gains/K4yaw Value of @f${k_4}_\psi@f$
@param Reference_Gains/X Gain for @f$x@f$ coordinate used to consider the reference acceleration of the path.
@param Reference_Gains/Y Gain for @f$y@f$ coordinate used to consider the reference acceleration of the path.
@param Reference_Gains/Z Gain for @f$z@f$ coordinate used to consider the reference acceleration of the path.
@param Lambda/X Variable used to limit control actions. It is not recommended to change.
@param Lambda/Y Variable used to limit control actions. It is not recommended to change.
@param Lambda/Z Variable used to limit control actions. It is not recommended to change.
@param Mass Bebop 2 drone mass.
@param Sigma Value of @f$ \sigma @f$
@param Disable_Commands Variable used to run the controller without sending velocity commands. This can be useful to check if the safe zone is working properly.

@file citc_controller_twist.yaml
@brief YAML file for CITC controller parameters, using velocity commands.

The following parameters can be configured in this file.
@param Gains/K1x Value of @f${k_1}_x@f$
@param Gains/K1y Value of @f${k_1}_y@f$
@param Gains/K1z Value of @f${k_1}_z@f$
@param Gains/K1yaw Value of @f${k_1}_\psi@f$
@param Gains/K2x Value of @f${k_2}_x@f$
@param Gains/K2y Value of @f${k_2}_y@f$
@param Gains/K2z Value of @f${k_2}_z@f$
@param Gains/K2yaw Value of @f${k_2}_\psi@f$
@param Gains/K3x Value of @f${k_3}_x@f$
@param Gains/K3y Value of @f${k_3}_y@f$
@param Gains/K3z Value of @f${k_3}_z@f$
@param Gains/K3yaw Value of @f${k_3}_\psi@f$
@param Gains/K4x Value of @f${k_4}_x@f$
@param Gains/K4y Value of @f${k_4}_y@f$
@param Gains/K4z Value of @f${k_4}_z@f$
@param Gains/K4yaw Value of @f${k_4}_\psi@f$
@param Reference_Gains/X Gain for @f$x@f$ coordinate used to consider the reference acceleration of the path.
@param Reference_Gains/Y Gain for @f$y@f$ coordinate used to consider the reference acceleration of the path.
@param Reference_Gains/Z Gain for @f$z@f$ coordinate used to consider the reference acceleration of the path.
@param Lambda/X Variable used to limit control actions. It is not recommended to change.
@param Lambda/Y Variable used to limit control actions. It is not recommended to change.
@param Lambda/Z Variable used to limit control actions. It is not recommended to change.
@param Mass Bebop 2 drone mass.
@param Sigma Value of @f$ \sigma @f$
@param Disable_Commands Variable used to run the controller without sending velocity commands. This can be useful to check if the safe zone is working properly.

@file max_speed.yaml
@brief YAML file to set the maximum velocity allowed for velocity commands.

The following parameters can be configured in this file.
@param Max_Speed/X Maximum velocity allowed in @f$x@f$ coordinate.
@param Max_Speed/Y Maximum velocity allowed in @f$x@f$ coordinate.
@param Max_Speed/Z Maximum velocity allowed in @f$x@f$ coordinate.
@param Max_Speed/Yaw Maximum angular velocity allowed for yaw angle.

@file normalize_angles.yaml
@brief YAML file to configure the parameters used to normalize speed for controllers using reference angles.

The following parameters can be configured in this file.
@param Normalize/Max_Tilt_Angle Maximum tilt angle. Used to normalize roll and pitch angles.
@param Normalize/Max_Vertical_Speed Maximum vertical speed. Used to normalize the control action in the @f$z@f$ coordinate
@param Normalize/Max_Rotation_Speed Maximum rotation speed. Used to normalize the yaw angle control action.

@file normalize_twist.yaml
@brief YAML file to configure the parameters used to normalize speed for controllers using velocity commands.

The following parameters can be configured in this file.
@param Normalize/Max_Horizontal_Speed Maximum horizontal speed. Used to normalize the speed in the @f$x@f$ and @f$y@f$ coordinates.
@param Normalize/Max_Vertical_Speed Maximum vertical speed. Used to normalize the control action in the @f$z@f$ coordinate
@param Normalize/Max_Rotation_Speed Maximum rotation speed. Used to normalize the yaw angle control action.

@file pid_controller_angles.yaml
@brief YAML file for PID controller parameters, using reference angles.

The following parameters can be configured in this file.
@param Gains/Px Value of @f$P_x@f$
@param Gains/Py Value of @f$P_y@f$
@param Gains/Pz Value of @f$P_z@f$
@param Gains/Pyaw Value of @f$P_\psi@f$
@param Gains/Dx Value of @f$D_x@f$
@param Gains/Dy Value of @f$D_y@f$
@param Gains/Dz Value of @f$D_z@f$
@param Gains/Dyaw Value of @f$D_\psi@f$
@param Gains/Ix Value of @f$I_x@f$
@param Gains/Iy Value of @f$I_y@f$
@param Gains/Iz Value of @f$I_z@f$
@param Gains/Iyaw Value of @f$I_\psi@f$
@param Limits/Ix Limit of the integral action for the coordinate @f$x@f$.
@param Limits/Iy Limit of the integral action for the coordinate @f$y@f$.
@param Limits/Iz Limit of the integral action for the coordinate @f$z@f$.
@param Limits/Iyaw Limit of the integral action for the yaw angle.
@param Reference_Gains/X Gain for @f$x@f$ coordinate used to consider the reference acceleration of the path.
@param Reference_Gains/Y Gain for @f$y@f$ coordinate used to consider the reference acceleration of the path.
@param Reference_Gains/Z Gain for @f$z@f$ coordinate used to consider the reference acceleration of the path.
@param Lambda/X Variable used to limit control actions. It is not recommended to change.
@param Lambda/Y Variable used to limit control actions. It is not recommended to change.
@param Lambda/Z Variable used to limit control actions. It is not recommended to change.
@param Mass Bebop 2 drone mass.
@param Disable_Commands Variable used to run the controller without sending velocity commands. This can be useful to check if the safe zone is working properly.

@file pid_controller_twist.yaml
@brief YAML file for PID controller parameters, using velocity commands.

The following parameters can be configured in this file.
@param Gains/Px Value of @f$P_x@f$
@param Gains/Py Value of @f$P_y@f$
@param Gains/Pz Value of @f$P_z@f$
@param Gains/Pyaw Value of @f$P_\psi@f$
@param Gains/Dx Value of @f$D_x@f$
@param Gains/Dy Value of @f$D_y@f$
@param Gains/Dz Value of @f$D_z@f$
@param Gains/Dyaw Value of @f$D_\psi@f$
@param Gains/Ix Value of @f$I_x@f$
@param Gains/Iy Value of @f$I_y@f$
@param Gains/Iz Value of @f$I_z@f$
@param Gains/Iyaw Value of @f$I_\psi@f$
@param Limits/Ix Limit of the integral action for the coordinate @f$x@f$.
@param Limits/Iy Limit of the integral action for the coordinate @f$y@f$.
@param Limits/Iz Limit of the integral action for the coordinate @f$z@f$.
@param Limits/Iyaw Limit of the integral action for the yaw angle.
@param Reference_Gains/X Gain for @f$x@f$ coordinate used to consider the reference acceleration of the path.
@param Reference_Gains/Y Gain for @f$y@f$ coordinate used to consider the reference acceleration of the path.
@param Reference_Gains/Z Gain for @f$z@f$ coordinate used to consider the reference acceleration of the path.
@param Lambda/X Variable used to limit control actions. It is not recommended to change.
@param Lambda/Y Variable used to limit control actions. It is not recommended to change.
@param Lambda/Z Variable used to limit control actions. It is not recommended to change.
@param Mass Bebop 2 drone mass.
@param Disable_Commands Variable used to run the controller without sending velocity commands. This can be useful to check if the safe zone is working properly.

@file proportional_controller.yaml
@brief YAML file for proportional controller parameters.

The following parameters can be configured in this file.
@param Gains/Px Value of @f$P_x@f$
@param Gains/Py Value of @f$P_y@f$
@param Gains/Pz Value of @f$P_z@f$
@param Gains/Pyaw Value of @f$P_\psi@f$
@param Reference_Gains/X Gain for @f$x@f$ coordinate used to consider the reference velocity of the path.
@param Reference_Gains/Y Gain for @f$y@f$ coordinate used to consider the reference velocity of the path.
@param Reference_Gains/Z Gain for @f$z@f$ coordinate used to consider the reference velocity of the path.
@param Disable_Commands Variable used to run the controller without sending velocity commands. This can be useful to check if the safe zone is working properly.

@file safe_zone.yaml
@brief YAML file to specify safe zone margins.

The following parameters can be configured in this file.
@param Safe_Zone/X Maximum position value allowed in the @f$x@f$ coordinate. If the drone reaches a position that exceeds this value, the drone will land.
@param Safe_Zone/Y Maximum position value allowed in the @f$y@f$ coordinate. If the drone reaches a position that exceeds this value, the drone will land.
@param Safe_Zone/Z Maximum position value allowed in the @f$z@f$ coordinate. If the drone reaches a position that exceeds this value, the drone will land.

@file square_root_controller.yaml
@brief YAML file for square root controller parameters.

The following parameters can be configured in this file.
@param Gains/Px Value of @f$P_x@f$
@param Gains/Py Value of @f$P_y@f$
@param Gains/Pz Value of @f$P_z@f$
@param Gains/Pyaw Value of @f$P_\psi@f$
@param Reference_Gains/X Gain for @f$x@f$ coordinate used to consider the reference velocity of the path.
@param Reference_Gains/Y Gain for @f$y@f$ coordinate used to consider the reference velocity of the path.
@param Reference_Gains/Z Gain for @f$z@f$ coordinate used to consider the reference velocity of the path.
@param Disable_Commands Variable used to run the controller without sending velocity commands. This can be useful to check if the safe zone is working properly.

@file topics.yaml
@brief YAML file to set the topics used by the nodes.

The following parameters can be configured in this file.
@param Topics/Command_Trajectory Topic used to send and receive the command trajectory between nodes.
@param Topics/Pose Topic used to receive the drone position data.
@param Topics/CMD_Vel Topic used to send the velocity commands to the drone.
@param Topics/TafeOff Topic used to send the takeoff command to the drone.
@param Topics/Land Topic used to send the land command to the drone.
@param Topics/CSV_Begin Topic used to communicate when the trajectory begins and the *data_to_csv* node should start saving the data.
@param Topics/CSV_End Topic used to communicate when the trajectory begins and the *data_to_csv* node should stop saving the data.

@file trajectory.yaml
@brief YAML file to set the trajectory parameters.

The following parameters can be configured in this file.
@param Trajectory/X_Initial Initial position in the @f$x@f$ coordinate.
@param Trajectory/Y_Initial Initial position in the @f$y@f$ coordinate.
@param Trajectory/Z_Initial Initial position in the @f$z@f$ coordinate.
@param Trajectory/X_Distance Distance that the trajectory travels in the @f$x@f$ coordinate.
@param Trajectory/Y_Distance Distance that the trajectory travels in the @f$y@f$ coordinate.
@param Trajectory/Z_Distance Distance that the trajectory travels in the @f$z@f$ coordinate.
@param Yaw_Enabled Variable that indicates whether to use or ignore the yaw angle, which faces the front of the trajectory.
@param Yaw_Offset Yaw angle offset in radians.

@file waypoint.yaml
@brief YAML file to configure trajectory times.

The following parameters can be configured in this file.
@param Waypoint/TimeBeforeTrajectory Time in seconds to let the drone reach the starting position.
@param Waypoint/TrajectoryTime Time in seconds of the trajectory.
@param Waypoint/MarginTime Time margin to let the drone reach the final position before landing.
@param Waypoint/DiffTime Refresh time of the Bebop Controller.

@namespace bebop_controller
@brief Namespace containing all the classes and functions of the Bebop Controller.

@namespace gazebo
@brief Namespace containing all the classes and functions used to get the drone position data from Gazebo.

@namespace plot
@brief Namespace that contains all the classes and functions used to generate graphs that display the test results.