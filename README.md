WE'LL BE SWITCHING TO https://github.com/RRL-ALeRT/ros2_kortex. THIS WORKS PERFECTLY FINE WITH ROS2 HUMBLE IF ANYBODY WANTS TO USE IT.
Nah, this one is more stable.

## Installation
```
pip install https://github.com/RRL-ALeRT/kinova_stuffs/raw/master/kortex_api-2.5.0.post6-py3-none-any.whl
```

## Fix for collections
```
pip install protobuf==3.19.4
```

## Launch
```
ros2 launch kortex_controller_py robot_launch.py
```
