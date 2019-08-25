## New Calibra Tool
### 介绍
这是一个基于 ROS 的单线激光和相机外参数自动标定代码。

### 特征
1. **支持多 April tag 格式**。 可以选择多 apriltag 的标定板 ( [kalibr_tag.pdf](doc/april_6x6_80x80cm_A0.pdf) ) 或者自己打印一个 apriltag 做标定板([apriltag.pdf](/doc/apriltags1-20.pdf))。
2. **支持多种相机模型**。**radial-tangential (radtan)** : (*distortion_coeffs*: [k1 k2 r1 r2]); **equidistant (equi)**:*distortion_coeffs*: [k1 k2 k3 k4]). More info please visit [kalibr website](https://github.com/ethz-asl/kalibr/wiki/supported-models).
3. **激光线自动检测，无须手标**。会自动提取激光线中落在标定板上的激光点，前提是标定板在激光正前方 120 度范围内，并且激光前方 2m 范围内只会存在一个连续的直线线段，所以请你在空旷的地方采集数据，不然激光数据会提取错误。
4. **利用标定板的边界约束，标定结果更加准确**。这个是隐藏的功能，暂时未对外开放接口，需要你修改代码。

### 编译方法

```c++
mkdir LaserCameraCal_ws
mkdir ../src
cd src
git clone https:/这个代码/
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### 运行

有两种运行模式，一种是先离线处理图像数据，再处理激光数据；一种是图像和激光数据一起处理。自己调试代码建议采用离线的方式，这样就不需要每次都重复运行图像识别，浪费时间。

无论采用那种模式运行，请实现配置好 config/calibra_config.yaml 文件，里面有**相机模型参数，rosbag 数据包的名字和保存路径**。相机模型支持不同类型，请选择对应的config.yaml.

#### 1. 离线模式

先运行 kalibr 检测图像二维码

```c++
cd LaserCameraCal_ws
source devel/setup.bash
roslaunch lasercamcal_ros kalibra_apriltag.launch 
```

再运行激光视觉外参数标定代码

```c++
roslaunch lasercamcal_ros calibra_offline.launch 
```

就能自动检测激光，并输出结果

验证结果

```c++
roslaunch lasercamcal_ros debug.launch 
rosbag play data.bag
```



## TODO

1. 支持修改相机采样间隔？？？ 还是说采用自动判断信息熵？
2. 

## RESULTS
![calibra](doc/calibra.gif)