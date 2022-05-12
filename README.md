# rsp-final
mobile robot navigation simulation with an ackermann-drive RC car

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

### Gazebo Simulation 

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

### Ackermann Plugin

### Ackermann EKF 

### Ball Chaser


 
