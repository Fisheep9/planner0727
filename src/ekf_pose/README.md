# 1.EKF IMU with Odomitry 

this package is a software development kit which is extend the hz .

# 2.How to Install
```
cd ${YOUR_WORKSPACE/src}
git clone git@github.com:SYSU-HILAB/ekf_pose.git
cd ..
catkin_make
source devel/setup.bash
roslaunch ekf nokov.launch 
```

# 3.How to use 

you should change three part:
**first** 
```
<remap from="~imu" to="/mavros/imu/data_raw"/>
```   
change the imu topic 
**second** 
```
<remap from="~bodyodometry" to="/vrpn_client_node/NMPC0/pose"/>
```
change the odometry topic 
**third**
```
<remap from="~ekf_odom" to="/ekf/ekf_odom"/>
```
change the topic name that you want send

