# nonlinear-surface-contactsensing

## Introduction

This project implements contact sensing algoritm of nonlinear surface from the intrinsic F/T(force/torque sensor) using ROS package based on c++. F/T sensor (Robotus RFT70) was used with CAN-bus communication. 

![image](https://user-images.githubusercontent.com/83193823/217184243-bfb2c8b7-5a4b-428c-8fd8-50251cf2832b.png)

< Screenshot of experiment of contact sensing algorithm >


![image](https://user-images.githubusercontent.com/83193823/217184134-e1804f01-4edf-4082-a1d7-007f0e7e390b.png)

< Real-time visualization of contact sensing using Rviz >


**Platform** : ROS, C++

## Usage

Install the ROS package:
```
cd catkin_ws/src
git clone https://github.com/downj0211/nonlinear-surface-contactsensing
cd ..
catkin_make
```

Then, CAN port should be initialized as:
```
./src/can_kernel_generate.sh
```

Finally, run the launch file:
```
roslaunch nonlinear_surface_contactsensing contactsensing_rviz.launch
```

