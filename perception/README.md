# Gimbal Perception
The following package allows to identify the sky line and a ground plane from outdoor images in real time. It takes a stabilized image as a reference and detects the movement of subsequent frames to send the signal to the servos.

## 1. Prerequisites
Download and install the [HELLO AI WORLD NVIDIA JETSON] repository (https://github.com/dusty-nv/jetson-inference). Focus mainly on [Semantic Segmentation with SegNet] (https://github.com/dusty-nv/jetson-inference/blob/master/docs/segnet-console-2.md) and how to run a demo with a live camera (https://github.com/dusty-nv/jetson-inference/blob/master/docs/segnet-camera-2.md). A Raspberry Pi camera (MIPI CSI camera) were used to record our demo video.
The pretrained network model is in this repository. It was used [Skyfinder Dataset] (https://cs.valdosta.edu/~rpmihail/skyfinder/) as dataset.

## 2. Build gimbal_perception package
Clone the respository in your default environment folder (in this example catkin_ws):
```
  cd ~/catkin_ws/src
  git clone https://github.com/hl49/uav_gimbal.git
  cd ../
  catkin_make
  source ~/catkin_ws/devel/setup.bash
```

## 3. Running the Gimbal Perception Node
3.1 Run the live camera segmentation network: 
```
./segnet.py --network=<model> csi://0                 # MIPI CSI camera
./segnet.py --network=<model> /dev/video0             # V4L2 camera
./segnet.py --network=<model> /dev/video0 output.mp4  # save to video file
```
3.2 Run the perception node:
```
rosrun gimbal_perception_node.py
```
