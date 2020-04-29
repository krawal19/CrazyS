# Introduction

## Crazyflie Simulation
The following simualtion is in gazebo and is sourced from [CrazyS](https://github.com/gsilano/CrazyS/)* repository. 

```
*NOTE: This repsoitory is used for the sole purpose of research and no part of it is used for commerical purposes.
```

To use the respository, follow the instructions as given in the CrazyS master README and build the package, recently they have added a swarm extension too [swarm feature](https://github.com/gsilano/CrazyS/issues/44).

After building the package, you can run the sample swarm by running the following command. 
```
$ roslaunch rotors_gazebo crazyflie2_swarm_hovering_example.launch
```
A custom swarm system can be made by changing the parameters in the launch file.
```
$ roslaunch rotors_gazebo crazyflie2_swarm_hovering_246.launch
```
## An overview of the launch file is as follows:
- Each crazyflie will be inside a group namespace,initialized with its parameters and initial spawn position is set. This part makes the spawn of crazyflie in the gazebo at the set initial location.  
```
<group ns="$(arg mav_name)_1">
  <!-- CRAZYFLIE_1 -->
    <include file="$(find rotors_gazebo)/launch/spawn_mav_crazyflie.launch">
      <arg name="namespace" value="$(arg mav_name)_1" />
      <arg name="mav_name" value="$(arg mav_name)" />
      <arg name="model" value="$(find rotors_description)/urdf/mav_generic_odometry_sensor.gazebo" />
      <arg name="enable_logging" value="$(arg enable_logging)" />
      <arg name="enable_ground_truth" value="$(arg enable_ground_truth)" />
      <arg name="enable_state_estimator" value="$(arg enable_state_estimator)" />
      <arg name="log_file" value="$(arg log_file)_1"/>
      <!-- Set the initial position -->
      <arg name="x" value="0.0"/>
      <arg name="y" value="0.0"/>
    </include>
```
- Here is a default position controller is defined which takes input on /trajectory topic. This can be replaced with a custom controller ([Issue Discussion](https://github.com/gsilano/CrazyS/issues/41)). Other controllers are available such as lee controller and mellinger controller in [CrazyS](https://github.com/gsilano/CrazyS/). The outputs of the control algorithm consist of the actuation commands (\omega_1, \omega_2, \omega_3 and \omega_4) sent to Gazebo (command/motor_speed) for the physical simulation and the corresponding graphical rendering, so to visually update the aircraft position and orientation (CrazyS/Master/ReadMe).

```
   <!-- The Crazyflie position controller -->
   <node name="position_controller_node" pkg="rotors_control" type="position_controller_node" output="screen">
      <param name="enable_state_estimator" value="$(arg enable_state_estimator)" />
      <param name="csvFilesStoring" value="$(arg csvFilesStoring)"/>
      <param name="csvFilesStoringTime" value="$(arg csvFilesStoringTime)"/>
      <param name="user_account" value="$(arg user_account)"/>
      <rosparam unless="$(arg enable_state_estimator)" command="load" file="$(find rotors_gazebo)/resource/controller_$(arg mav_name).yaml" />
      <rosparam if="$(arg enable_state_estimator)" command="load" file="$(find rotors_gazebo)/resource/controller_$(arg mav_name)_with_stateEstimator.yaml" />
      <rosparam command="load" file="$(find rotors_gazebo)/resource/$(arg mav_name).yaml" />
   </node>
```
- The node hovering_example publishes the desired trajectory on /trajectory topic, it is not the trajectory controller. Thus spline methods (rotors_gazebo/src/library/spline_trajector_generator.cpp) can be used for trajectory generation for the crazyflie, it publishes states on /drone_state topic.
```
   <!-- <node name="hovering_example" pkg="rotors_gazebo" type="hovering_example" output="screen" /> -->
```
- This node can be used for taking waypoint inputs from a text file as given in rotors_gazebo/resource/example_waypoints.txt file. This node and the hover node can be used to effectively run drone to follow trajectory using txt file and spline methods. The args format for waypoint_publisher is x y z t1 t0. The (x,y,z) are the final position of the drone. t0 and t1 is the wait time after which the drone will start, generall from 0  to t (time to start).
```
   <node name="waypoint_publisher" pkg="rotors_gazebo" type="waypoint_publisher" output="screen" args="0 0 1 0 6"/>
```
- Below nodes publishes robot state and joint state and remaps the odometery. 
```
   <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
   <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
   <node name="quaternion_to_rpy" pkg="rotors_gazebo" type="quaternion_to_rpy" output="screen" >
       <remap from="odometry" to="odometry_sensor1/odometry" />
   </node>
  </group>
```
- Multiple such groups can be added in the same launch file for getting swarm of drones.

## This repository contains the following features for simulation:
- Single crazyflie drone in simulation
- Swarm of crazyflie drone in simualtion
- Contoller for crazyflie 
- Waypoint controller

## Various task can be performed ahead :

- Add a controller for hovering and waypoint.
- Run the swarm system using trajectory csv file.
- Add vicon system to gazebo for tracking the swarm system and use as given in crazySwarm repository.
- Use any swarm planning algorithm for testing using the crazy swarm system. [OMPL](https://ompl.kavrakilab.org/) can be used for adding planning algorithms.
- The simulation of multiple crazyfile is slow in gazebo, so task is to find the a new simulator which doesn't consume high load for large swarm system or make changes to the exisiting gazebo simualtion.


## Crazyflie2 Keyboard control
A python script under /cf_hardware/keyboard_control.py can be used to control crazyflie2 using the keyboard.

The sytem requires proper drives to be installed, it requires the following python dependencies:
- cflib - the crazyflie python API
- MotionCommander [link](https://github.com/bitcraze/crazyflie-lib-python/blob/master/cflib/positioning/motion_commander.py)

To use the script to control the drone over the keyboard:
- Power on the crazyflie and set it on a level surface
- Plug in the crazyradio PA to the computer
- Ensure the URI string in the script appropriately matches the interface id, channel and speed parameters for the crazyflie
- Run the script, python keyboard_control.py
- Enjoy flying with ease!