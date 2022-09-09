# truck_sim
autonomous mining truck:

Preview.
<div align="center"><img src="https://github.com/mdaa-amine/truck_sim/blob/main/truck-sim.gif" /></div>

* Clone , build & Configure : # Ros <ROS-DISTRO> : Melodic and Pyhton2.
```
git clone https://github.com/mdaa-amine/truck_sim
cd truck_sim && catkin_make 
source devel/setup.bash
rosdep check --from-paths src --ignore-src --rosdistro <ROS-DISTRO>
rosdep install --from-paths src --ignore-src --rosdistro <ROS-DISTRO> -y

```
* dependencies:
```
sudo apt-get install ros-<ROS-DISTRO>-effort-control*
sudo apt-get install ros-<ROS-DISTRO>-velocity-control*
sudo apt-get install ros-<ROS-DISTRO>-joint-state-*
sudo apt-get install ros-<ROS-DISTRO>-hector-gazebo-plugins
sudo apt-get install ros-<ROS-DISTRO>-position-controllers
```
* As we use gps install the hector plugin.
```
sudo apt-get install ros-<ROS-DISTRO>-hector-gazebo-plugins
```
* launch the simulation:
```
roslaunch truck_description spawn_robot.launch
```

![truck](https://user-images.githubusercontent.com/60377645/164751588-f38fe524-27f7-4fee-8e50-36f4b56f4f19.png)


* launch go to point while avoiding obstacles node (New Terminal).
```
cd truck_sim
source devel/setup.bash
cd src/truck_control/src/
chmod u+x *
rosrun truck_control go_to_points_while_avoiding_obstacles.py
```
Video Simulation.
  
<div align="center"><img src="https://github.com/mdaa-amine/truck_sim/blob/main/truck-sim.gif" /></div>

* The Path.
  
<div align="center"><img src="https://user-images.githubusercontent.com/60377645/189372133-5692c20f-a4f5-4d52-9b7e-443d9f1e14a2.png" /></div>

* The Control Commande.
  
<div align="center"><img src="https://user-images.githubusercontent.com/60377645/189370706-8712d44e-3e5e-442f-b329-696a821464d4.png" /></div>

## Build Status
|    | Melodic |
|--- |--- |
| steer_bot | [![Build Status](https://travis-ci.com/srmainwaring/steer_bot.svg?branch=develop)]() |
