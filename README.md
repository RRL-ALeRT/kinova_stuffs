## Installation
```
pip install https://github.com/RRL-ALeRT/kinova_stuffs/raw/master/kortex_api-2.5.0.post6-py3-none-any.whl
```

## Fix for collections
```
sudo nano ~/.local/lib/python3.10/site-packages/google/protobuf/internal/containers.py
```
add
```
if sys.version_info.major == 3 and sys.version_info.minor >= 10:
    import collections
    setattr(collections, "MutableMapping", collections.abc.MutableMapping)
    setattr(collections, "MutableSequence", collections.abc.MutableSequence)
```
below
```
import sys
```

## Launch
```
ros2 launch kortex_controller_py robot_launch.py
```
