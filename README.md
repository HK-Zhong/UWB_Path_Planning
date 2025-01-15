# 0 项目说明
![](./img/demo.png)
- 在配置文件中写明了不同 AprilTag 在世界坐标系下的位置。实时订阅所有无人机的实时位置。分配目标点到所有无人机上，无人机利用 ego_planner 完成运动规划，抵达目标点附近，检测到 AprilTag 位姿后发送出来，由于有障碍物的存在，不同无人机到达目标点的时间有所不同，接收到检测到的 AprilTag 后可视化其位置，并重新分配目标点到所有无人机，进行动态调整。在所有无人机完成了检测任务后飞回原点。
- gazebo rotos中要加载的资源非常多，因此整体非常非常卡顿，不确定提高电脑性能能否提高运行速度，需要具体测试。
# 1 下载配置说明
- 环境说明：`Ubuntu20.04`、`ROS1.0 Noetic`
- 依赖下载
```shell
# 1 下载行人模拟依赖
git clone https://github.com/robotics-upo/lightsfm.git
cd lightsfm
make
sudo make install

# 2 下载 AprilTag 依赖
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
cmake -B build -DCMAKE_BUILD_TYPE=Release
sudo cmake --build build --target install
```
- 下载编译
```shell
# 务必保持这个目录，后续一键运行脚本是基于这个目录的
mkdir -p ~/JKW_PROJECT/collaborative_detect_sim_ws/src
cd ~/JKW_PROJECT/collaborative_detect_sim_ws/src
git clone https://github.com/HK-Zhong/collaborative_detect.git
cd ~/JKW_PROJECT/collaborative_detect_sim_ws/

# 编译较为耗时，性能不够可能会自动关机，在 NUC 上编译时出现了这种情况
catkin_make
```

- chmod 一键运行脚本
```shell
chmod +x ~/JKW_PROJECT/collaborative_detect_sim_ws/src/collaborative_detect/utils/shfiles/*.sh
```
# 2 代码模块说明
## 2.1 apriltag_ros
- 用于实时检测AprilTag并估计其位姿的功能模块，具体用法参考[官方例程](https://github.com/AprilRobotics/apriltag_ros)
- 使用 `continuous_detection.launch` 进行连续检测，订阅传感器图片话题，以话题的形式输出AprilTag的位姿。
- 需要修改的参数包括
    - `<remap from="image_rect" to="$(arg camera_name)/$(arg image_topic)" />`：表示订阅的无人机拍摄的图像话题名，它这里定义了全局参数的形式组合成具体的相机话题名，可以直接把它改成具体的话题名
    - `<remap from="camera_info" to="$(arg camera_name)/camera_info" />`：相机的内参话题，通常就是以camera_info结尾的一些话题，它这里定义了全局参数的形式组合成具体的相机话题名，同样可以直接把它改成具体的话题名。
    - `<group ns="ardrone_1">`：命名空间，对检测出发布的位姿和标记图片话题加上前缀，尤其是在多机仿真的情况下避免话题冲突
        - `/ardrone_1/tag_detections`：表示检测到的一些AprilTag在相机坐标系下的位姿及编号的信息，该话题的类型是`AprilTagDetectionArray`，是april_tag_ros包自定义的消息类型，具体格式可以参考`apriltag_ros`包下的`msg`文件夹
        - `/ardrone_1//tag_detections_image`：就是在检测到的图片上打上标记后以图片的形式发布出来
    - `<rosparam command="load" file="$(find apriltag_ros)/config/settings.yaml"/>`与`<rosparam command="load" file="$(find apriltag_ros)/config/tags.yaml"/>`是两个配置文件
        - `settings.yaml`：用于说明待检测的AprilTag是哪个family的，AprilTag有多个family，每个family有多个标签，通常只需要修改第一个参数`tag_family:'tag36h11' `
        - `tags.yaml`：用于标记需要检测的标志及其尺寸，这个`size`参数非常重要，是待检测二维码的实际大小（不包括外围白边），需要实际测量。
            ```yaml
            standalone_tags:
            [
                {id: ID, size: SIZE}
            ]
            ```
        - 仿真的 AprilTag 生成参考 [gazebo_apriltag](https://github.com/koide3/gazebo_apriltag)

- 这个代码并没有在这里启动 apriltag_ros，而是一起放到了 ego_planner中的launch文件里

## 2.2 gazebo_sfm_plugin
- 该功能包为在 gazebo 环境中添加移动的人，移动轨迹可自行指定，具体用法可参考[官方例程](https://github.com/robotics-upo/gazebo_sfm_plugin)
- 需要在 rotors_simulator 中加载的word中添加具体的移动小人模型。
## 2.3 rotors_simulator & mav_comm
- 该功能包提供了无人机和gazebo word的仿真，默认启动运行的launch文件在`rotors_simulator/rotors_gazebo/launch` 中，其中`multi_uav.launch`为启动多架无人机的脚本，需要增加新的无人机时可以按照如下方式添加新的无人机，注意需要设置好其中的`<arg name="x" value="40.0"/>`、`<arg name="y" value="10.0"/>`及`<arg name="z" value="1"/>`，还有`<node name="waypoint_publisher" pkg="rotors_gazebo" type="waypoint_publisher" output="screen" args="40 10 1 180 5"/>`，其中`40 10 1`是坐标位置，后面两个分别是偏航角和响应时间，一般不用改。需要减少无人机时注释掉某个`<group ...> </group>` 代码块即可。
    ```launch
    <group ns="ardrone_1">
      <include file="$(find rotors_gazebo)/launch/spawn_mav.launch">
        <arg name="number" value="1"/>
        <arg name="mav_name" value="ardrone" />
        <arg name="model" value="$(find rotors_description)/urdf/mav_with_vi_sensor.gazebo" />
        <arg name="enable_logging" value="$(arg enable_logging)" />
        <arg name="enable_ground_truth" value="$(arg enable_ground_truth)" />
        <arg name="log_file" value="ardrone1"/>
        <arg name="x" value="40.0"/>
        <arg name="y" value="-10.0"/>
        <arg name="z" value="1"/>
        <arg name="yaw" value="3.14159"/>
      </include>
      <node name="lee_position_controller_node" pkg="rotors_control" type="lee_position_controller_node" output="screen">
        <rosparam command="load" file="$(find rotors_gazebo)/resource/lee_controller_ardrone.yaml" />
        <rosparam command="load" file="$(find rotors_gazebo)/resource/ardrone.yaml" />
        <remap from="odometry" to="odometry_sensor1/odometry" />
      </node>
      <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
      <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
      <node name="waypoint_publisher" pkg="rotors_gazebo" type="waypoint_publisher" output="screen" args="40 -10 1 180 5"/>
    <node pkg="rotors_gazebo" type="DistanceCalculator.py" name="drone1_distance_calculator">
      <param name="drone_id" value="1"/>
      <param name="num_drones" value="$(arg num_drones)"/>
    </node>
    </group>
    ```
- 可根据实际情况选择合适的gazebo world，在`rotors_simulator/rotors_gazebo/launch/multi_uav.launch`文件中的`<arg name="world_name" default="jkw_dynamic_2"/>`修改，具体的gazebo世界在`rotors_simulator/rotors_gazebo/worlds`中，本文设置的默认采用的世界模型是`jkw_dynamic_2`，是一个比较大的动态世界，`jkw_dynamic`是一个比较小的动态世界，也可以自定义一些其它的世界模型，需要在world文件中添加`<plugin name="ros_interface_plugin" filename="librotors_gazebo_ros_interface_plugin.so"/>`以确保无人机能够正确加载起飞。
    - apriltag的设置实例
    ```xml
    <model name='Apriltag36_11_00000'>
        <pose>-1 47 0.6 0 1.45955 0</pose>
        <scale>1 1 1</scale>
        <link name='main'>
          <pose>-1 47 0.6 0 1.45955 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
    ```
    - 移动行人的设置实例，其中`<ignore_obstacles>`中添加的是环境中的其它障碍物
    ```xml
    <actor name="actor1">
      <pose>28 -13 1.25 0 0 0</pose>
      <skin>
        <filename>walk-red.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walking">
        <filename>walk-red.dae</filename>
        <scale>1.000000</scale>
        <interpolate_x>true</interpolate_x>
      </animation>

      <plugin name="actor1_plugin" filename="libPedestrianSFMPlugin.so">
        <velocity>0.9</velocity>
        <radius>0.4</radius>
        <animation_factor>5.1</animation_factor>
        <people_distance>6.0</people_distance>
        <!-- weights -->
        <goal_weight>2.0</goal_weight>
        <obstacle_weight>80.0</obstacle_weight> <!--10.0-->
        <social_weight>15</social_weight> <!--2.1-->
        <group_gaze_weight>3.0</group_gaze_weight>
        <group_coh_weight>2.0</group_coh_weight>
        <group_rep_weight>1.0</group_rep_weight>
        <ignore_obstacles>
          <model>asphalt_plane</model>
          <model>ground_plane</model>
          <model>jersey_barrier_clone</model>
          <model>jersey_barrier_0_clone</model>
          <model>jersey_barrier_1_clone</model>
          <model>jersey_barrier_2</model>
          <model>jersey_barrier_3</model>
          <model>jersey_barrier_4</model>
          <model>jersey_barrier_5</model>
          <model>jersey_barrier_0</model>
          <model>jersey_barrier_8</model>
          <model>jersey_barrier_9</model>
          <model>jersey_barrier_9_clone</model>
          <model>jersey_barrier_9_clone_clone</model>
          <model>jersey_barrier_10</model>
          <model>jersey_barrier_10_clone</model>
          <model>jersey_barrier_10_clone_clone</model>
          <model>jersey_barrier_10_clone_clone_clone</model>
          <model>jersey_barrier_11</model>
          <model>Construction Barrel</model>
          <model>Construction Barrel_clone</model>
          <model>Construction Barrel_0</model>
          <model>Construction Barrel_0_clone</model>
          <model>Construction Barrel_0_clone_clone</model>
          <model>Construction Barrel_0_clone_clone_clone</model>
          <model>Construction Barrel_2</model>
          <model>Construction Barrel_2_clone</model>
          <model>Construction Barrel_2_clone_0</model>
          <model>Construction Barrel_3</model>
          <model>Construction Barrel_3_clone</model>
          <model>asphalt_plane_0_clone</model>
          <model>asphalt_plane_0_clone_clone</model>
          <model>asphalt_plane_0_clone_clone_clone</model>
          <model>asphalt_plane_1</model>
          <model>asphalt_plane_1_clone</model>
          <model>asphalt_plane_1_clone_clone</model>
          <model>asphalt_plane_1_clone_clone_clone</model>
          <model>asphalt_plane_1_clone_clone_clone_0</model>
          <model>asphalt_plane_1_clone_clone_clone_0_clone</model>
          <model>asphalt_plane_1_clone_clone_clone_clone</model>
          <model>asphalt_plane_1_clone_clone_clone_clone_0</model>
          <model>asphalt_plane_1_clone_clone_clone_clone_clone</model>
          <model>asphalt_plane_1_clone_clone_clone_clone_clone_0</model>
          <model>asphalt_plane_1_clone_clone_clone_clone_clone_clone</model>
          <model>asphalt_plane_2</model>
          <model>asphalt_plane_3</model>
          <model>asphalt_plane_3_clone</model>
          <model>asphalt_plane_4</model>
          <model>asphalt_plane_4_clone</model>
          <model>ardrone_1</model>
          <model>ardrone_2</model>
          <model>ardrone_3</model>
          <model>ardrone_4</model>
          <model>bus</model>
          <model>bus_clone</model>
          <model>bus_0</model>
          <model>oak_tree</model>
          <model>oak_tree_0</model>
          <model>oak_tree_0_clone</model>
          <model>oak_tree_0_clone_clone</model>
          <model>gas_station</model>
          <model>suv</model>
          <model>suv_clone</model>
          <model>warehouse_robot</model>
          <model>Apriltag36_11_00000</model>
          <model>Apriltag36_11_00001</model>
          <model>Apriltag36_11_00002</model>
          <model>Apriltag36_11_00003</model>
          <model>Apriltag36_11_00004</model>
          <model>Apriltag36_11_00005</model>
          <model>Apriltag36_11_00006</model>
          <model>Apriltag36_11_00007</model>
          <model>Apriltag36_11_00008</model>
        </ignore_obstacles>
        <trajectory>
          <cyclic>true</cyclic>
          <waypoint>28 -13 1.25</waypoint>
          <waypoint>28 2 1.25</waypoint>
        </trajectory>
      </plugin>
    </actor>
    ```
- 由于太卡，一般不开启gazebo GUI 界面，在`rotors_simulator/rotors_gazebo/launch/multi_uav.launch`文件中的`<arg name="gui" value="false"/>`修改是否开启 gui。
## 2.4 utils
- 此处更详细的说明清参考 real_drone 分支
- `apriltag_detect.py`：由于 apriltag_ros 功能包发布的位姿话题是在相机坐标系下的，而我们希望得到的marker位姿是在世界坐标系下，因此需要进行位姿转换并发布出去`/ardrone_{i}/tag_pose`，类型为`DetectionPose`，是自定义类型的话题，在`utils/msg/DetectionPose.msg`中可查看，主要就是包含了一个tag的ID（与前文所说的AprilTag family下所说的ID是一个意思）
- `target_distribution.py`：读取配置文件`utils/config/tags.yml`表示所有标志物的大概位置，同时实时订阅三架无人机的实时位置，将问题建模为mTSP问题，利用google的OR-Tools工具求解该问题，分配 tag 目标位置，同时订阅`/ardrone_{i}/tag_pose`，在检测到后实时调整所有的tag目标分配。
    - 配置文件`utils/config/tags.yml`的格式为具体的无人机id和其位置数据。这里的AprilTag位置可以在gazebo布置环境的时候读取，尽量设置为整数的位置。
- 其中`apriltag_detect.py`的启动放在了`ego_planner`下的`single.launch`中了，`target_distribution.py`的启动放在了`utils/launch/target_distribution.launch`中，要修改的参数包括配置文件的路径`config_file_path`以及无人机的数量`num_drones`
## 2.5 ego_planner
- 对[ego_planner](https://github.com/ZJU-FAST-Lab/ego-planner)进行修改以支持rotors仿真的接入，以及达到航点后的巡查策略。
- 虽然在仿真过程中需要涉及到集群规划，但在这里并没有用ego_swarm，而是对ego_planner进行一定的修改。
- 启动`~/project/collaborative_detect_sim_ws/src/collaborative_detect/ego-planner/src/planner/plan_manage/launch/swarm.launch`可以实现多无人机规划的仿真
    - `swarm.launch`的仿真中接入了`single.launch`，在`single.launch`中启动了`ego_planner`、`apriltag_ros`、`apriltag_detect`以及`odom_visualization`
- `~/project/collaborative_detect_sim_ws/src/collaborative_detect/ego-planner/src/planner/plan_manage/launch/rviz.launch`是多无人机运动规划的可视化配置，已经配置好了最多五架无人机规划的可视化
- 如果要增加无人机，可以在`ego-planner/src/planner/plan_manage/launch/swarm.launch`文件中添加如下代码块，其中的`/ardrone_4`记得都要修改为对应的。
```shell
<include file="$(find ego_planner)/launch/single.launch">
    <arg name="prefix" value="ardrone_4"/>
    <arg name="odom_topic" value="/ardrone_4/odometry_sensor1/odometry"/>
    <arg name="depth_topic" value="/ardrone_4/vi_sensor/camera_depth/depth/disparity"/>
    <arg name="cmd_topic" value="/ardrone_4/command/pose"/>
    <arg name="max_vel" value="$(arg max_vel)"/>
    <arg name="max_acc" value="$(arg max_acc)"/>

    <arg name="map_size_x" value="$(arg map_size_x)"/>
    <arg name="map_size_y" value="$(arg map_size_y)"/>
    <arg name="map_size_z" value="$(arg map_size_z)"/>

    <arg name="camera_name" default="/ardrone_4/vi_sensor/left" />
    <arg name="image_topic" default="image_raw" />
</include>
```
# 3 添加或减少无人机数量时需要做的修改
- 需要修改`rotors_simulator/rotors_gazebo/launch/multi_uav.launch`文件中的无人机数量，具体参考上面所说，记住要确保无人机的初始位置是没有障碍物的，可以先打开 gui 观察一下再设置不打开。减少无人机直接注释掉某个无人机代码块
- 需要修改`ego-planner/src/planner/plan_manage/launch/swarm.launch`文件中的无人机代码块，具体参考上面相应板块所说。减少无人机直接注释掉某个无人机代码块
- 需要修改`utils/launch/flag_publish.launch`中的`<arg name="drone_nums" default="4"/>`
- 需要修改`utils/launch/target_distribution.launch`中的`<param name="num_drones" value="4"/>`

# 4 使用说明
```shell
# 启动仿真环境
cd ~/JKW_PROJECT/collaborative_detect_sim_ws/src/collaborative_detect/utils/shfiles/

# 打开 rotors 仿真环境，默认是不开启 gui 的，如需开启，修改参数，详情阅读相应章节
# 很容易冲突出错，关掉后等一会再重启
./1_start_rotors.sh

# 同一目录下开启新的终端，启动规划节点
./2_start_planner.sh

# 同一目录下开启新终端，启动节点位置发送节点
./3_flag_publisher.sh

# 同一目录下开启新终端，启动话题记录节点，录制的 rosbag 就在当前的目录下，因为非常卡，所以无法当时录制视频，录制数据后回放
./4_record.sh

# 同一目录下开启新终端，启动 rviz 可视化节点，如果非常卡，这个可视化也可以不启动，过一段时间启动一下查看是否完成任务后立马关闭
./5_start_rviz.sh

# 同一目录下开启新终端，开启目标点分配程序，输出搜索完成就是完成了搜索返回原点了，等待返回原点才是真的任务完成了。
./6_start_distribution.sh

```
# 5 录制数据重放说明
- 录制数据在 `~/JKW_PROJECT/collaborative_detect_sim_ws/src/collaborative_detect/utils/shfiles/` 目录下，先利用 `./5_start_rviz.sh` 打开 RVIZ 可视化终端，然后开启录屏软件，随后利用 `rosbag play xxxxx.bag` 指令播放数据。
