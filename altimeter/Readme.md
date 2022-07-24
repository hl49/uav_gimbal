# Altimeter
The current package measures the altitude above sea level (m.s.l).

## 1. Prerequisites
The packaged is build to read an BMP180 Sensor. Software requirements are described in the [root folder](README.md) of this repository

## 2. Build altimeter package
Clone the repository and build through catkin_make:

```
  cd ~/catkin_ws/src
  git clone https://github.com/hl49/uav_gimbal.git
  cd ../
  catkin_make
  source ~/catkin_ws/devel/setup.bash
```

Allow execution permission to the *altimeter_node.py* file
```
  cd ~/catkin_ws/src/uav_gimbal/altimeter/scripts
  chmod +x altimeter_node.py
```

## 3. Running the Altimeter node
3.1 In order to run the altimeter node run the command:
```
  rosrun altimeter altimeter_node.py
```

3.2 The BMP180 data is publish in the default topic */altimeter/BMP180*, which is a custom topic with the structure described bellow:
```
  std_msgs/Header header
  float32 altitude
  float32 temperature
```
The *altitud* is given in meters and the *temperature* in Celsius degrees
