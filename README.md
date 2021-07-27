# lidar_driver

### build steps.
1. create ws/src folder.
>mkdir -p lidar_driver_ws/src
2. clone repogitory.
>cd lidar_driver_ws/src  
git clone -b \<branch> \<url>
3. build.
>cd lidar_driver_ws  
colcon build  
4. running
>cd lidar_driver_ws  
>cp ./src/lidar_driver/ros_wrapper/config/horizon_sensor_config.yaml .  
>vi ./horizon_sensor_config.yaml  
> (edit file)  
> ---> host_ip: your PC ip address.  
> ---> sensor_ip: livox device ip address.  
>source install/setup.bash  
>ros2 run lidar_ros_driver lidar_driver_node --ros-args --params-file ./horizon_sensor_config.yaml -r __ns:=/livox_a  
5. rviz
>rviz2 -f livox_a

Add Botton. By topic /livox_a/livox/cloud   PointCloud2  
Displays PointCloud2 Topic Reliability Policy: Best Effort  
