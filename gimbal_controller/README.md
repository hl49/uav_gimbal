# Gimbal Controller
The current package controls the orientation of a gimbal system built with 2 servos [Dynamixel xl430-w250-t](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/), the hardware interface used to communicate the servos with a pc is the [opencr](https://emanual.robotis.com/docs/en/parts/controller/opencr10/)

## 1. Prerequisites
The servos driver and communication packages are required to run the current module. The sofware dependencies are described in the [dynamixel interface respository](https://github.com/csiro-robotics/dynamixel_interface).
The current package requires the sensor information from BNO055 IMU, the sensor driver is found in the[
ros_imu_bno055](https://github.com/RoboticArts/ros_imu_bno055) repository.

## 2. Build gimbal_controller package
Clone the respository in your default environment folder (in this example catkin_ws):
```
  cd ~/catkin_ws/src
  git clone https://github.com/hl49/uav_gimbal.git
  cd ../
  catkin_make
  source ~/catkin_ws/devel/setup.bash
```

Allow execution permission to the *gimbal_controller_node.py* file
```
  cd ~/catkin_ws/src/uav_gimbal/gimbal_controller/scripts
  chmod +x gimbal_controller_node.py
```
## 3. Running the Gimbal Controller Node
3.1 Connect the *Opencr* to the pc, and turn on the servo power supply.
3.2 Run the custom launch file required to initialize the servo driver:
```
roslaunch gimbal_controller dynamixel_interface_controller.launch
```
3.3 Run the controller node:
```
rosrun gimbal_controller gimbal_controller_node.py
```
3.4 Position reference from input topics

The gimbal controller keeps the initial position until a reference position is sent through [perception node](perception) and [ros_imu_bno055](https://github.com/RoboticArts/ros_imu_bno055). The gimbal_controller_node subcribes to the topics `/perception_rpy` and `/imu/data`. The first topic provides reference position relative to the skyline processed by the perception node, and the second topic contains the imu measurements published by the ros_imu_bno055 node.

