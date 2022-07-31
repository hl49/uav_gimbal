# Gimbal Perception
The following package allows to identify the sky line and a ground plane from outdoor images in real time. It takes a stabilized image as a reference and detects the movement of subsequent frames to send the signal to the servos.

## 1. Prerequisites
Download and install the repositories [HELLO AI WORLD NVIDIA JETSON](https://github.com/dusty-nv/jetson-inference) for running the segmentation neural network and [ROS DEEP LEARNING](https://github.com/dusty-nv/ros_deep_learning) for ROS1 interface. Focus mainly on [Semantic Segmentation with SegNet](https://github.com/dusty-nv/jetson-inference/blob/master/docs/segnet-console-2.md) and how to run a demo with a [Semantic Segmentation live camera](https://github.com/dusty-nv/jetson-inference/blob/master/docs/segnet-camera-2.md). A Raspberry Pi camera (MIPI CSI camera) was used to record our demo video.
The pretrained network model is in the [model](./model) folder. It was used [Skyfinder Dataset](https://cs.valdosta.edu/~rpmihail/skyfinder/) as dataset.

## 2. Build gimbal_perception package
Clone the respository in your default environment folder (in this example catkin_ws):
```
  cd ~/catkin_ws/src
  git clone https://github.com/hl49/uav_gimbal.git
  cd ../
  catkin_make
  source ~/catkin_ws/devel/setup.bash
```
2.1 Include pretrained network modified:
First, copy the content from the [model](./model) folder into the *jetson-inference/python/examples* directory. Then, modify the content of the segnet.ros1.launch according to the following lines:
```
TODO
```


## 3. Running the Gimbal Perception Node
3.1 Run the live camera segmentation network: 
```
roslaunch ros_deep_learning segnet.ros1.launch
```
3.2 Run the perception node:
```
rosrun perception perception_node.py
```
