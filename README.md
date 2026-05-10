# fast_mk_gsmini

## Introduction

In this directory, we realize the maximal marker reading frequence of gelsightmini about 25hz, providing the source code and compilation files for find_marker.so (located at find_marker_generator/srclib/find_marker.so). 

## Requirements

1. install ROS [here.](https://wiki.ros.org/ROS/Installation)

2. python environment setup
```python
$(which python) -m pip install -r requirements.txt
```

## find_marker.so file generalization
We recommand that you'd better generalize your find_marker.so file in your personal computer to match your python version.
See more details in [README.md](find_marker_generator/README.md)

## Test
in tumx one:
```python
roscore
```
in tumx two:
```python
python main.py
```
then you should see


## Acknowledgement
Our work is built upon [reactive_diffusion_policy](https://github.com/xiaoxiaoxh/reactive_diffusion_policy) and [gs_sdk](https://github.com/joehjhuang/gs_sdk). Thanks for their great work!