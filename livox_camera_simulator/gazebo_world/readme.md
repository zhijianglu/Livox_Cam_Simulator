```
    # First run 
    export GAZEBO_MODEL_PATH=your_path/velo2cam_simulation/gazebo_models

    # Test zed2 camera 
    roslaunch gazebo_world zed2.launch

    # calibration system of vlp16-zed2
    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=vlp16 K:=0 zed2_x:=0.5 zed2_y:=0.0 zed2_z:=1.2 zed2_roll:=0.0 zed2_pitch:=0.0 zed2_yaw:=1.5707963267949

    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=vlp16 K:=0 zed2_x:=-0.5 zed2_y:=0.0 zed2_z:=1.2 zed2_roll:=0.0 zed2_pitch:=0.0 zed2_yaw:=1.5707963267949

    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=vlp16 K:=0 zed2_x:=1.0 zed2_y:=0.1 zed2_z:=1.3 zed2_roll:=-0.10 zed2_pitch:=0.1 zed2_yaw:=2.0

    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=vlp16 K:=0 zed2_x:=1.0 zed2_y:=0.1 zed2_z:=1.0 zed2_roll:=0.10 zed2_pitch:=0.1 zed2_yaw:=2.0

    # calibration system of vlp32-zed2
    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=hdl32 K:=0 zed2_x:=0.5 zed2_y:=0.0 zed2_z:=1.2 zed2_roll:=0.0 zed2_pitch:=0.0 zed2_yaw:=1.5707963267949

    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=hdl32 K:=0 zed2_x:=-0.5 zed2_y:=0.0 zed2_z:=1.2 zed2_roll:=0.0 zed2_pitch:=0.0 zed2_yaw:=1.5707963267949

    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=hdl32 K:=0 zed2_x:=1.0 zed2_y:=0.1 zed2_z:=1.3 zed2_roll:=-0.10 zed2_pitch:=0.1 zed2_yaw:=2.0

    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=hdl32 K:=0 zed2_x:=1.0 zed2_y:=0.1 zed2_z:=1.0 zed2_roll:=0.10 zed2_pitch:=0.1 zed2_yaw:=2.0

    # calibration system of vlp64-zed2
    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=hdl64 K:=0 zed2_x:=0.5 zed2_y:=0.0 zed2_z:=1.2 zed2_roll:=0.0 zed2_pitch:=0.0 zed2_yaw:=1.5707963267949

    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=hdl64 K:=0 zed2_x:=-0.5 zed2_y:=0.0 zed2_z:=1.2 zed2_roll:=0.0 zed2_pitch:=0.0 zed2_yaw:=1.5707963267949

    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=hdl64 K:=0 zed2_x:=1.0 zed2_y:=0.1 zed2_z:=1.3 zed2_roll:=-0.10 zed2_pitch:=0.1 zed2_yaw:=2.0

    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=hdl64 K:=0 zed2_x:=1.0 zed2_y:=0.1 zed2_z:=1.0 zed2_roll:=0.10 zed2_pitch:=0.1 zed2_yaw:=2.0

```

rosbag
```
    # 场景0
    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=hdl32

    # 场景1
    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=hdl32 zed2_x:=-0.8 zed2_yaw:=1.4707963267949

    # 场景2
    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=hdl32 zed2_z:=1.5 zed2_roll:=-0.1 zed2_pitch:=0.0 zed2_yaw:=1.6707963267949

    # 场景3
    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=hdl32 zed2_x:=1.5 zed2_z:=1.5 zed2_roll:=-0.1 zed2_pitch:=0.0 zed2_yaw:=1.6707963267949

    #hdl64
    # 场景0
    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=hdl64 velodyne_z:=1.5

    # 场景1
    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=hdl64 velodyne_z:=1.5 zed2_x:=-0.8 zed2_yaw:=1.4707963267949

    # 场景2
    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=hdl64 velodyne_z:=1.5 zed2_z:=1.5 zed2_roll:=-0.1 zed2_pitch:=0.0 zed2_yaw:=1.6707963267949

    # 场景3
    roslaunch gazebo_world lidar_zed2_scence.launch velodyne:=hdl64 velodyne_z:=1.5 zed2_x:=1.5 zed2_z:=1.5 zed2_roll:=-0.1 zed2_pitch:=0.0 zed2_yaw:=1.6707963267949

    rosbag record --duration 10 -O `rospack find lidar_camera_calibration`/data/simulation/rosbag/vlp16_zed2_k0_0.bag /tf_static /left_camera/camera_info /left_camera/image_raw /right_camera/camera_info /right_camera/image_raw

    rosbag record --duration 10 -O `rospack find lidar_camera_calibration`/data/simulation/rosbag/vlp16_zed2_k0_0.bag /velodyne_points /tf_static /left_camera/camera_info /left_camera/image_raw /right_camera/camera_info /right_camera/image_raw
```