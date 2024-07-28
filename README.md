## Real-World-Flight

### 1  Quick Start

**Step 1:** Initialize the repository and compile:

```shell
git@github.com:Fisheep9/planner0727.git
cd Real-World-Flight
catkin_make
```



**Step 2:** Initialize the controller and communicator:

If this is your **FIRST** flight, please read the [Parameters Modification(Section 4)](### 4  Modify the Parameters before the First Run) first.

```shell
./shfiles/init.sh
```

More details are in the Section **Init module**.



**Step 3: **Send the trigger to take off.

```sh
#!/bin/bash
source ../devel/setup.bash
rostopic pub -1  /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
```

**OR** you can choose to run the shell file which is more conveniently:

```sh
./shfiles/takeoff.sh
```



**Step 4:** Setup your remote controller to:

- Switch the **fifth** channel to offboard mode
- Switch the **sixth** channel to high-end mode
- Disable the **emergency** stop
- Set the **throttle** to the mid-position



**Step5: ** Start the planner:

```sh
./shfiles/ego.sh
```

More details are in the Section **Ego module**.



**Step 6: ** Land and Finish:

Firstly, send the trigger to land:

```sh
#!/bin/bash
source ../devel/setup.bash
rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2"
```

**OR** more conveniently, you can run the shell:

```sh
./shfiles/land.sh
```

Then switch to the **emergency stop** and unplug the battery.



### 2 Init Module

Firstly, you need to **TEST** and ensure that the basic packages are running well.

If this is your first flight, please modify the following:

- Comment out the command in the **FOURTH** line and test if `init.sh` runs well.
- Substitute `"(your own password)"` with your user password, such as "1234" or "abcd".
- Ensure that the VICON server IP is `10.1.1.198`.

The details of `init.sh` are as follows:

```sh
#!/bin/bash
source ../devel/setup.bash
echo "your own password" | sudo -S chmod 777 /dev/tty*
# roslaunch mavros px4.launch & sleep 3
rosrun mavros mavcmd long 511 31 10000 0 0 0 0 0;
roslaunch vrpn_client_ros sample.launch server:=10.1.1.198 & sleep 3
roslaunch odom_converter converter.launch & sleep 3
roslaunch ekf nokov.launch & sleep 3
roslaunch px4ctrl run_ctrl.launch 
```

Then, if all the programs are running smoothly, uncomment the **FOURTH** line and rerun the init module.



### 3  Ego module

```sh
#!/bin/bash
source ../devel/setup.bash
roslaunch ego_planner single_run_in_exp.launch 
```

You can modify the target points in `src/planner/plan_manage/launch/single_run_in_exp.launch` as following:

```xml
<arg name="point_num" value="4" />

<arg name="point0_x" value="5.0" />
<arg name="point0_y" value="0.0" />
<arg name="point0_z" value="0.8" />

<arg name="point1_x" value="2.5" />
<arg name="point1_y" value="0.0" />
<arg name="point1_z" value="0.96" />

<arg name="point2_x" value="2.5" />
<arg name="point2_y" value="0.0" />
<arg name="point2_z" value="1.0" />

<arg name="point3_x" value="2.5" />
<arg name="point3_y" value="0.0" />
<arg name="point3_z" value="1.0" />
```





### 4  Modify the Parameters before the First Run

#### 4.1 px4ctrl

The main launch file `px4ctrl/launch/run_ctrl.launch`  is as following:

```xml
<launch>
  <node pkg="px4ctrl" type="px4ctrl_node" name="px4ctrl" output="screen">
    <remap from="~odom" to="/ekf/ekf_odom" />
    <!-- <remap from="~cmd" to="/setpoints_cmd" /> -->
  	<!-- msg type: quadrotor_msgs::PositionCommand -->
    <remap from="~cmd" to="/position_cmd" /> 
    <rosparam command="load" file="$(find px4ctrl)/config/ctrl_param_fpv.yaml" />
  </node>
</launch>
```

Before your first flight, you need to modify the config file in `src/px4ctrl/config/ctrl_param_fpv.yaml`.

The following parameters are of significant importance:

- `mass`: 1.60 	# kg
- `takeoff_height`: 0.8     # meter (relative height)
- `hover_percentage`: 0.25    # % percentage (derived from manual mode)
- `takeoff_land_speed`: 0.3    # m/s

#### 4.2 VICON

The use of the VICON package depends on the pre-built rigid body model in the VICON server. You **MUST** modify the `rigid_body` argument to match your specific rigid body name.

```xml
<launch>
    <arg name="capture_num_" value="1" />
    <arg name="rigid_body" value="am_test" />
	
    <node pkg="odom_converter" type="odom_converter" name="odom_converter" output="screen">
        <param name="capture_num" value="$(arg capture_num_)" />
        <param name="cap_num" value="$(arg capture_num_)" type="int" />
        <remap from="~/pose0" to="/vrpn_client_node/$(arg rigid_body)/pose" />
        <remap from="~/twist0" to="/vrpn_client_node/$(arg rigid_body)/twist" />
    </node>
</launch>
```





