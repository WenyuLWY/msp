# msp

## Step

### download the simulation environment and path planning module

To use with AEDE:

```
git clone -b noetic https://github.com/WenyuLWY/autonomous_exploration_development_environment.git
```

Use robot model:

```
git clone -b main https://github.com/WenyuLWY/autonomous_exploration_development_environment.git
```

### download the exploration framework

```
git clone https://github.com/WenyuLWY/tare_planner.git
```

### run

Start the simulation environment and msp module:

```
roslaunch msp run.launch
```

and start the exploration module:

```
roslaunch tare_planner explore.launch
```

gazebo /root/catkin_ws/src/msp_sim/script/building.world

apt list --installed | grep -i gazebo
dpkg-query -W -f='${binary:Package}\t${Version}\t${Status}\n' | grep -i gazebo
dpkg-query -W -f='${binary:Package}\n' | grep -i gazebo

msp_sim_turtlebot3_2d is newest


roslaunch msp_sim msp_sim_husky_3d.launch
