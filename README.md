# YOLOv8 ROS2 Wrapper
## System Requirement
1. Python >= 3.8
2. ROS2 Foxy/Humble
3. Ubuntu 20/22

## Installation 
1. First install [PyTorch](https://pytorch.org/) on the machine (for Python and C++ LibTorch). For NVIDIA Jetson, follow the instruction at https://forums.developer.nvidia.com/t/installing-pytorch-for-cuda-10-2-on-jetson-xavier-nx-for-yolov5/183868/2 for installation.
2. Obtain a pretrained model in torchscript format (.torchscript/.torchscript.pt). Follow instructions in YOLOv8 [export](https://docs.ultralytics.com/modes/export/).

## Run 
1. Clone this project to your ROS2 workspace
```
cd ~/ros2_ws/src
git clone https://github.com/joewong00/yolov8-ros2-wrapper.git yolov8_wrapper
```
2. Ensure both `torchscript` model and a class lists file (`coco.names`) is present, modify the path to both of the files in the launch file.
3. Install ROS dependencies
```
cd ../
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```
4. Build the package
```
colcon build --symlink-install
```
5. Run the detector
```
ros2 launch yolov8_wrapper detect.launch.py
```

## Reference
https://github.com/ultralytics/ultralytics/tree/main/examples/YOLOv8-LibTorch-CPP-Inference