# CamLaserCalibraTool
Extrinsic Calibration of a Camera and 2d Laser

## code build and compile:

> mkdir -p calibra_ws/src
>
> cd calibra_ws/src 
>
> git clone https://github.com/MegviiRobot/CamLaserCalibraTool
>
> cd ..
>
> catkin_make

## Calibration Method
As Fig show, we use an apriltag marker as a calibration board. These 2d laser scan points should on the calibration planar. After computing the marker pose $T_{cm}$ in the camera frame by solvePnp, we can get the planar equation in the camera frame. Then using these points on the planar to constraint the extrinsic $T_{cl}$.
![calibra_method](https://img-blog.csdnimg.cn/20181214120913798.png)

## Calibration Step
### 1. Data Collection 
Prepare a calibration plate as follow figure shown. Then place the calibration plate in front of the camera and laser, and the laser scan should fall on the calibration plate. Record all image and laser data with rosbag. Constantly adjust the plate pose, each time you change the pose, please keep still for more than 2 seconds, collect data with about 10 different pose (of course, the more the better).
![calibra_board](https://img-blog.csdnimg.cn/20181214140949954.png)

### 2. Prepare config.yaml parameters 
### 3. Save Marker Pose
Run the following code to get the marker pose:

> source calibra_ws/devel/setup.bash
>
> roslaunch apriltags_ros example.launch
>
> rosbag play your_bag.bag

### 4. Run Calibration
> roslaunch lClibra example.launch 

After running the code, the tool will automatically detect each moment that camera do not move. Once detected, the laser data will be drawn into an image. You can select the lasers that just fall on the calibration board from the image. Notice: Don't choose points that is not on the plane. Then press any key to continue, as shown below:
![select_data](https://img-blog.csdnimg.cn/20181214142017919.png)

### 5. Calibration Evaluation
#### evaluation method 1

> roslaunch lClibra example.launch
>
> rosbag play your_bag.bag
 
we project the laser point to the image plane, and you will get a image show  as follow:
![evaluation1](https://img-blog.csdnimg.cn/20190530183443953.png)

#### evaluation method 2
A python visualization tool is prepared in the folder config_and_tool, which will plot the point cloud data and plane at each static moment. You can check whether the laser data falls on the plane to evaluate the result. The figure below shows the laser point cloud and the plane of the calibration plate at a certain moment.

![evaluation](https://img-blog.csdnimg.cn/20181214142436227.png)

## Authors
1. Yijia He, you can find the chinese version [in his homepage](https://blog.csdn.net/heyijia0327/article/details/85000943), if you have any question, please contact heyijia at megvii dot com
2. XiZhen Xiao
3. Xiao Liu, [his homepage](http://www.liuxiao.org/)

## Ref:
2004, Qilong Zhang, Extrinsic Calibration of a Camera and Laser Range Finder (improves camera calibration).
