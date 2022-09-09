# truck_sim
autonomous mining truck:

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
![truck](https://github.com/mdaa-amine/truck_sim/blob/main/truck-sim.mp4)
