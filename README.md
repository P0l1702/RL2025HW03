# RL2025HW03 :package:
Fly your drone

## Getting Started :hammer:
Make sure you have installed `QGROUNDCONTROL` available [here](https://qgroundcontrol.com)

```shell
cd /ros2_ws/src
git clone https://github.com/P0l1702/RL2025HW03.git
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.16.0
cd ..
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs
git checkout release/1.16
```
Copy the corresponding files from the folder `RL2025HW03/PX4-Autopilot` in the original  `PX4-Autopilot` folder. Then in the`/ros2_ws` fold execute:
```shell
colcon build 
source install/setup.bash
```

## Usage :white_check_mark:

 ## **1.Fly pizza drone** :pizza:
Open `QGROUNDCONTROL` then in a new terminal:
```shell
cd /ros2_ws/src/PX4-Autopilot/
make px4_sitl gz_pizza_drone
```
A new gazebo simulation will open and from now you can fly your pizza drone by using the sticks. 

 ## **2.Force land revisited** :arrows_clockwise:
Open `QGROUNDCONTROL` then in a new terminal:
```shell
cd /ros2_ws/src/PX4-Autopilot/
make px4_sitl gz_pizza_drone
```
In a second terminal run the following command:
```shell
cd /ros2_ws/src
./DDS_run.sh
```
And in a third terminal launch:
```shell
cd /ros2_ws
ros2 run force_land force_land
```
From now, you can takeoff your drone. The landing procedure will activate only once, after that the threshold of 20m is surpassed. 

To test the correct functioning, it is advisable to do the takeoff directly above 20 m, and then resume control by using the sticks. 

## **3. Flying in offboard mode** :airplane:
Open `QGROUNDCONTROL` then in a new terminal:
```shell
cd /ros2_ws/src/PX4-Autopilot/
make px4_sitl gz_pizza_drone
```
In a second terminal run the following command:
```shell
cd /ros2_ws/src
./DDS_run.sh
```
And in a third terminal launch:
```shell
cd /ros2_ws
ros2 run offboard_rl execute_trajectory
```
Then you can see, the pizza drone following the trajectory.
### Note:
In the repository, also bag files are available in the `bag` folder. :coffee:
