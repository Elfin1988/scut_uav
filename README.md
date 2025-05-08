# scut_uav
基于ROSneotic+ubuntu20.04+ROTORS功能包构建出的small_project
1.项目说明
1.1仿真环境说明

仿真软件平台： o 操作系统：Ubuntu20+ROS+Gazebo

o ROS版本： ROSNoetic

o Gazebo11版本
1.2仿真比赛场地说明：

起飞区域：无人机比赛的起始点。

S 绕行区域：划定S形路径区域，要求无人机按照指定轨迹飞行。

视觉降落区域：使用视觉标识作为指示的降落区域。

二维码辅助定位降落区域：设置二维码图案作为辅助定位信息的降落区域。

ID 信标识别区域 (前/后)：设置ID信标作为赛事过程中信息采集对象。

穿行区域：由多个门框构成的区域，需要无人机自主穿梭飞行。
1.3仿真任务说明

1.起飞区域 无人机从指定起飞区域自主起飞并稳定悬停，不限制飞行方向

2.穿行区域 无人机自主穿过穿行区域内的所有障碍物门框。

3.无人机自主飞抵ID信标获取区域且自动记录前后两个二维码信息 ID信标识别区域 (前)

4.二维码辅助降落区域 无人机自主飞抵二维码辅助定位降落区域，并精准降落在指定区域内。

5.S 绕行区域 (绕行中途视觉降落区域） 无人机自主沿S形轨迹飞行并完整飞出S绕行区域。

6.无人机自主飞抵视觉降落区域，并精准降落在指定区域内。 降落至初始起飞点

7.无人机自主识别并降落至起飞点
2.使用方法
2.1下载镜像文件

链接: https://pan.baidu.com/s/1SVeYIC31pZpeUbTG4tygWw?pwd=hetb

提取码: hetb

scut-uav 为无人机镜像系统，包含无人机功能包（catkin_ws）
2.2更改内容

1.增加qr_code_detector文件夹以及其中的文件

2.更改/home/slz/catkin_ws/src/rotors_simulator/rotors_gazebo/src/hovering_example.cpp其中的内容

3.更改lee_position_controller.cpp和lee_position_controller.h的文件

4.更改柱子上面的二维码
2.3运行

''' roscore

rosrun qr_code_detector qr_code_detector.py

roslaunch rotors_gazebo mav_hovering_example_with_vi_sensor.launch mav_name:=hummingbird world_name:=scut_uav '''
3.作者

SCUT小飞棍战队
