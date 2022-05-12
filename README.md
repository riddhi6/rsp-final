# rsp-final
Mobile robot navigation simulation with an Ackermann-drive RC car. 

## Description
As opposed to the standard differential drive, Ackermann cars are controlled using steering angle and rear wheel velocity. The front axle of an Ackermann car is actuated by a servo to set the steering angle of the car. The rear axle is actuated by a throttle motor that corresponds to the rear wheel velocity. 

<img src="https://www.researchgate.net/profile/Jiri-Krejsa/publication/224184352/figure/fig2/AS:302837121732616@1449213404660/Ackermann-steering-principle.png" alt="Ackermann Drive" width="400"/>


## Installation
For Keyboard Control:
`sudo apt-get install ros-melodic-teleop-twist-keyboard 
`
For sensors
`sudo apt-get install ros-melodic-gps-umd`
`sudo apt-get install ros-melodic-gazebo-plugins`
`cd src 
git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo `

For movement
`sudo apt-get install ros-melodic-navigation `
`sudo apt-get install ros-melodic-robot-localization `

## Use 

### Gazebo Simulation Launch Files
Use keyboard as teleop:

`roslaunch ackermann_simulation key_simulation.launch`

Make robot chase a ball:

`roslaunch ackermann_simulation ball_simulation.launch`

press enter in main gazebo terminal for gazebo to start

### Gazebo Simulation Manual Start

Terminal 1:

``roslaunch ackermann_car_description ackermannCar_startup.launch``

Terminal 2:

``rosrun teleop_twist_keyboard teleop_twist_keyboard.py``
or 
`roslaunch ball_chaser ball_chaser.launch`

Terminal 3:
`roslaunch ackermann_ekf gazebo_ackermann_ekf.launch`

Terminal 4:
`rostopic echo odometry/filtered`

## Package Descriptions
### Ackermann Car Description
Robot description for a mobile robot using ackermann-drive. Due to the inherent complexities of an Ackermann drive, the simulated robot has not been modeled to exactly match the real mobile robot. The real robot includes four continuous wheel joints, two revolute hinge joints between the front wheels and bearing that mimic each other in the front axle and two fixed hinge joints on the rear axle. In contrast, the simulated robot is modeled as four continuous wheel joints with a revolute hinge joint between each wheel and axle bearing, all independent of each other to match the gazebo plugin described below. The full robot_description with the robot and sensors integrated is uploaded from car.urdf.xacro

URDF files include:
* Car chassis
* Car wheel
* Car transmission
* IMU model with Gazebo plugin
* GPS model with Gazebo plugin
* Kinect model with Gazebo plugin
* Hokuyo model with Gazebo plugin
* Ackermann drive model plugin


### Ackermann Plugin
Source code for the model plugin used to simulate the ackermann drive mechanism of the robot in the Gazebo envrionment. The [original plugin](https://github.com/froohoo/ackermansteer) was heavily edited to work for Gazebo 9 and ROS-Melodic. The plugin requires the ackermann car to be modeled as a robot with four independent continuous wheel joints each connected to an independent revolute hinge joint. The plugin also includes a PID controller that obtains the true steering angle and wheel velocity of the car from Gazebo and compares those values to the desired steering angle and rear wheel velocity from the command velocity twist it's subscribed to. Using these target values, the plugin then sets the effort of each joint.

### Ackermann EKF 
Package that includes launch files and parameters needed to run Extended Kalman Filter for state estimation of the mobile robot location. The relevant nodes come from the [robot_localization package](http://wiki.ros.org/robot_localization). The EKF uses IMU data, gps data, and its own filtered state estimate via the navsat_transform_node to produce the estimate position and orientation. 

### Ball Chaser
Package that uses the image data from the robot's kinect to chase a ball. When a ball is detected in the camera image, the robot command velocity will be set so that the mobile robot moves towards the ball. The ball can be set in the gazebo GUI environment using translate mode 

### Ackermann Simulation
Package that contains all launch-files needed to run a full ackermann simulation with ball chasing or teleop keyboard control. 

 
