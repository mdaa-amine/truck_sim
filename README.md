# truck_sim
autonomous mining truck:

* Clone , build & Configure :

```
git clone https://github.com/mdaa-amine/truck_sim
cd truck_sim
catkin_make
rosdep check --from-paths src --ignore-src --rosdistro melodic
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
source devel/setup.bash
```

* dependencies:
```
sudo apt-get install ros-melodic-effort-control*
sudo apt-get install ros-melodic-velocity-control*
sudo apt-get install ros-melodic-joint-state-*
sudo apt-get install ros-melodic-hector-gazebo-plugins
sudo apt-get install ros-melodic-position-controllers
```
* launch the simulation:
```
roslaunch truck_description spawn_robot.launch
```
![image](https://user-images.githubusercontent.com/60377645/116946948-0331b700-ac6b-11eb-90c1-0fe60e5ecfb1.png)

* launch go to point while avoiding obstacles node.
```
rosrun controller go_to_points_while_avoiding_obstacles.py
```
