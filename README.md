# Ackermann Localization Simulation
Mobile robot navigation simulation with an Ackermann-drive RC car. 

## Description
This project provides a simulation in Gazebo of mobile robot localization with an Ackermann-drive based RC Car (RedCat Rampage). As opposed to differential drive, Ackermann cars are controlled using steering angle and rear wheel velocity. The front axle of an Ackermann car is positioned by a servo through a series of linkages to set the steering angle of the car. The wheels on the rear axle are actuated by a motor to set wheel velocity. 

<img src="https://www.researchgate.net/profile/Jiri-Krejsa/publication/224184352/figure/fig2/AS:302837121732616@1449213404660/Ackermann-steering-principle.png" alt="Ackermann Drive" width="400"/>

## System Requirements

* Ubuntu 18.04 (Bionic Beaver)
* ROS Melodic 

## Installation
This project was built on and intended for use with ROS-Melodic and Gazebo 9 (the default version of Gazebo that comes with Melodic). It is also dependent on multiple packages available that are not present in the default installation of ROS Melodic. 


### ROS  Dependencies

```
sudo apt-get install ros-melodic-teleop-twist-keyboard
sudo apt-get install ros-melodic-gazebo-plugins
sudo apt-get install ros-melodic-navigation 
sudo apt-get install ros-melodic-gps-umd
sudo apt-get install ros-melodic-robot-localization 
```

### Create a workspace
```
mkdir -p catkin_ws/src
cd ~/catkin_ws/src`
git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo 
git clone https://github.com/riddhi6/rsp-final
cd ..
catkin build
source devel/setup.bash
```
## Launching Packages 

### Start the robot in the World

Launch the ackermann mobile robot and world. 

Terminal 1:
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch ackermann_car_description ackermannCar_startup.launch
```
### Launch the Extended Kalman Filter Package for robot localization
Terminal 2:
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch ackermann_ekf gazebo_ackermann_ekf.launch
```
### View the estimated robot position in a new terminal.
Terminal 3: 

```
cd ~/catkin_ws
source devel/setup.bash
rostopic echo odometry/filtered
```
### Move the Robot 
In a new terminal launch use either option 1 or option 2 to move the robot. Option 1 allows you to teleop control the robot using your keyboard (press the indicated keys in the specified terminal). In option 2, if you position the white ball in the gazebo environment (using translated mode of the gazebo GUI) in the view of the mobile robot's camera, the robot will follow the ball. View the video provided for more instructions on how to use Option 2. 

Terminal 4:
```
cd ~/catkin_ws
source devel/setup.bash
```
Option 1: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
 
Option 2: `roslaunch ball_chaser ball_chaser.launch`



### Gazebo Simulation Launch Files
As an alternative, you can use one of the two provided launch files in the ackermann simulation package to replicate the prior detailed commands. 
#### Launch robot with keyboard teleop

Launch the Gazebo/Rviz simulation to control the mobile robot with teleop keyboard control in the relevant terminal and view the EKF state estimate of robot positon. 

`roslaunch ackermann_simulation keyboard_control.launch`

#### Launch robot with ball chasing 

Launch the Gazebo/Rviz simulation. In gazebo, enter translate mode and use your mouse to pick up and move the white ball within sight of the mobile robot's kinect camera. The robot will then move towards the ball. In one of the xterm terminals you can view the EKF estimate of robot position. 

`roslaunch ackermann_simulation follow_ball.launch`

## Package Descriptions
### Ackermann Car Description
Robot description for a mobile robot using ackermann-drive. Due to the inherent complexities of an Ackermann drive, the simulated robot has not been modeled to exactly match the real mobile robot. The real robot includes four continuous wheel joints, two revolute hinge joints between the front wheels and bearing that mimic each other in the front axle and two fixed hinge joints on the rear axle. In contrast, the simulated robot is modeled as four continuous wheel joints with a revolute hinge joint between each wheel and axle bearing, all independent of each other to match the gazebo plugin described below. 

<img src="https://github.com/riddhi6/rsp-final/blob/main/docfiles/cartf.png" alt="Car in Rviz" width="300"/>

The full robot_description with the robot and sensors integrated is uploaded from car.urdf.xacro. 

URDF files include:
* Car chassis
* Car wheel
* Car transmission
* IMU model with Gazebo plugin
* GPS model with Gazebo plugin
* Kinect model with Gazebo plugin
* Hokuyo model with Gazebo plugin
* Ackermann drive model plugin

<img src = "https://github.com/riddhi6/rsp-final/blob/main/docfiles/rvizgazebo.png" alt="car sensors" width=600>

### Ackermann Plugin
Source code for the model plugin used to simulate the ackermann drive mechanism of the robot in the Gazebo envrionment. The [original plugin](https://github.com/froohoo/ackermansteer) was edited to work for Gazebo 9 and improve plugin performance. The plugin requires the ackermann car to be modeled as a robot with four independent continuous wheel joints each connected to an independent revolute hinge joint. The plugin also includes a PID controller that obtains the true steering angle and wheel velocity of the car from Gazebo and compares those values to the desired steering angle and rear wheel velocity from the x linear and z angular velocities from the twist it's subscribed to. Using these target values, the plugin then sets the effort of each joint.

### Ackermann EKF 
Package that includes launch files and parameters needed to run Extended Kalman Filter for state estimation of the mobile robot location. The relevant nodes come from the [robot_localization package](http://wiki.ros.org/robot_localization). The EKF uses IMU data, gps data, and its own filtered state estimate via the navsat_transform_node to produce the estimate position and orientation. 

<img src="https://github.com/riddhi6/rsp-final/blob/main/docfiles/ekf0.png" alt="ekf0" width = 400/><img src="https://github.com/riddhi6/rsp-final/blob/main/docfiles/ekf1.png" alt="ekf1" width = 400/>

### Ball Chaser
Package that uses the image data from the robot's kinect to chase a ball. When a ball is detected in the camera image, the robot command velocity will be set so that the mobile robot moves towards the ball. The ball can be set in the gazebo GUI environment using translate mode.

<img src="https://github.com/riddhi6/rsp-final/blob/main/docfiles/ball0.png" width = 400/><img src="https://github.com/riddhi6/rsp-final/blob/main/docfiles/ball2.png" width = 400/>

### Ackermann Simulation
Package that contains all launch-files needed to run a full ackermann simulation with ball chasing or teleop keyboard control. 

## References
* ackermannplugin
* robot localization
* udacity ball chasing 
 
