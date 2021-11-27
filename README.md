# RTKGPS_IMU_Encoders_Plot_trajectory_using_ROS

In this project, we Processed data from RTK-GPS, IMU and encoders to plot the trajectory of a field robot in RViz. The pose and twist of the robot are also published using an Odometry ROS message.

In the PLOTS pdf, Qn 2, we see that when we use IMU+encoders, we get errors in yaw angle due to noise in gyroscope readings.  Hence, the trajectory looks like a polygon with sharp changes in yaw which influences the (x,y) position at the subsequent time step(t+1).

Whereas in Qn3, when only GPS is used, we get a straight line trajectory.

Refer to RVIZ_PLOTS pdf for trajectory plots.

## Instructions to get the trajectory plots from the ROSBAG of the real field robot:
1. Extract ```coding_ex_1.zip``` in your ```~/catkin_ws/src``` folder.
2. ```cd ~/catkin_ws``` and then ```catkin_make```.
3. Run ```roscore``` in Terminal #1.
4. In Terminal #2, ```cd ~/catkin_ws/src/coding_ex_1/scripts``` then ```rosbag play -l 2020-08-31-00-01-17.bag```
5. In Terminal #3, ```rosrun coding_ex_1 odometry_example_node.py```
6. In Terminal #4, ```rviz``` and set "Keep" parameter to 10,000 to see the trajectory. Other parameters in Rviz can be seen as follows:
 ![alt text](./qn2_plot_rviz_parameters_cropped.png?raw=true "Parameters")
