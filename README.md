**总体环境：**
本项目基于 Ubuntu 20.04 和 ROS Noetic 进行开发和测试.

**安装依赖**
```bash
sudo apt-get install libglfw3-dev libglew-dev
```

**克隆marsim仓库**
```bash
mkdir -p ~/ catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/MARSIM.git（HTTPS）或者用git clone git@github.com:hku-mars/MARSIM.git(SSH)
cd ..
source devel/setup.bash
```

**编译**
```bash
catkin_make
roslaunch test_interface single_drone_avia.launch
```

**环境配置**

 注意修改第 Utils /odom_visualization/src/odom_visualization.cpp第183 行加 个 '/'
 ```bash
 pathROS.header.frame_id = quad_name + "/world";
 ```
 
**仿真中运行 fast-lio2**

先创建你自己的工作空间:
```bash
mkdir -p ~/ catkin_ws /src
 ```
进入工作空间的src目录，在src下

**1. 安 装 Fast -LIO2**
```bash
git clone https://github.com/hku-mars/FAST_LIO.git
 ```
同理，也可用ssh的
```bash
git clone git@github.com:hku-mars/FAST_LIO.git
 ```
**克 隆 ikd -Tree**
```bash
cd ~/ catkin_ws /src
git clone https://github.com/hku-mars/ikd-Tree.git 或
git clone git@github.com:hku-mars/ikd-Tree.git
 ```
把这个ikd-Tree里面的ikd-Tree文件夹复制到前面 FAST_LIO 中 的 include 文 件 夹 下 

**克 隆 livox_ros_driver,安装到src下即可**
```bash
git clone https://github.com/Livox-SDK/livox_ros_driver.git 或
git clone git@github.com:Livox-SDK/livox_ros_driver.git
 ```

**安装Livox-SDK和Livox-SDK2**
在主文件夹新建文件夹ThirdParty保存第三方库
在ThirdParty下
```bash
git clone https://github.com/Livox-SDK/Livox-SDK.git  或
git clone git@github.com:Livox-SDK/Livox-SDK.git 
cd Livox-SDK
cd build && cmake ..
make
sudo make install
 ```

**安装Livox SDK2**
一样在ThirdParty下
```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git 或
git clone git@github.com:Livox-SDK/Livox-SDK2.git
cd ./ Livox -SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
 ```

**编译工作空间**
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch fast_lio mapping_marsim.launch
 ```

**启动masrim仿真环境**
```bash
roslaunch test_interface single_drone_avia.launch
 ```
可以看到 rviz, 里面有仿真中的点云和里程计



**MARSIM使用的是点云地图文件（.pcd） 作为环境，这意味着：可以创建新的场景的点云文件（.pcd），
然后修改launch文件中的map_name参数指向新的pcd文件，即可更改世界环境**

**使用python生成点云文件
在主目录创建项目文件夹**
```bash
mkdir -p ~/pcd_generation
cd ~/pcd_generation
 ```

**编写python脚本**
```bash
cd~/pcd_generation
nano generate_pcds.py
 ```

# 注意！！！确保在虚拟环境中运行python脚本
python generate_pcds.py
确保catkin_ws/src/MARSIM/map_generator/resource$里面有新生成的pcd文件，如果没有，复制进去
回到catkin_ws/src/MARSIM/test_interface/launch，把single_drone_avia.launch里面的map_name参数指向新的pcd文件，
例如， 把LibraryLG_01cutoff_sor.pcd换成sparse_forest.pcd

**启动验证**
```bash
  cd ~/catkin_ws
  roslaunch test_interface single_drone_avia.launch 
   ```
  
  
#创建运动控制脚本
  八字轨迹：伯努利双纽线（Lemniscate of Bernoulli）
Lemniscate.py


**启动验证**
```bash
  cd ~/catkin_ws
  roslaunch test_interface single_drone_avia.launch
  roslaunch fast_lio mapping_marsim.launch
 ```
# 注意！！！确保在虚拟环境中运行python脚本
```bash
  python3 scripts/Lemniscate.py
   ```
  
  
  
**数据收集对比,两种方法：1.在线实时录制(node)     2.离线录制（rosbag）**

**1. 用node**
  
  
**创建python脚本，编写⼀个轻量级的订阅节点，在程序运⾏时直接将数据写⼊⽂本⽂件**
fast_lio_recorder_estimated.py
fast_lio_recorder_groundtruth.py 


#启动
```bash
roslaunch test_interface single_drone_avia.launch

roslaunch fast_lio mapping_marsim.launch
 ```

**确保是在虚拟环境下**

```bash

python3 scripts/fast_lio_recorder_groundtruth.py 
python3 scripts/fast_lio_recorder_estimated.py 
python3 scripts/Lemniscate.py
  ```
  
**用evo对比数据**

**全局⼀致性评估 (APE-Absolute Pose Error)**

评估整条轨迹与真值的重合程度，检查 SLAM 系统是否存在全局地图变形。
```bash

evo_ape tum ground_truth_tum.txt estimated_tum.txt  -va --plot  --save_plot ape_plot.pdf

evo_ape tum ground_truth_tum.txt estimated_tum.txt -va --save_results ape_corridor_01.zip  #(保存为zip）

evo_ape tum ground_truth_tum.txt estimated_tum.txt -va 2>&1 | tee ape_results_01.txt
 ```


**⾥程计漂移评估 (RPE - Relative Pose Error)**

评估系统在局部运动中的精度，即 “ 每⾛⼀⽶或每过⼀秒产⽣的误差 ”
```bash
evo_rpe tum ground_truth_tum.txt estimated_tum.txt -va --plot 
evo_rpe tum ground_truth_tum.txt estimated_tum.txt -va --plot        --save_results rpe_translational.zip 
evo_rpe tum ground_truth_tum.txt estimated_tum.txt -va --plot        --save_plot rpe_translational_plot.pdf
  ```
       
# 将详细结果输出到文本文件
```bash
evo_rpe tum ground_truth_tum.txt estimated_tum.txt -va 2>&1 | tee rpe_translational_results.txt
 ```


# 评估旋转误差（角度制）并保存完整结果
```bash
evo_rpe tum ground_truth_tum.txt estimated_tum.txt -va --plot --pose_relation angle_deg \
        --save_results rpe_rotational.zip \
        --save_plot rpe_rotational_plot.pdf
 ```

# 将旋转误差详细结果输出到文本文件
```bash
evo_rpe tum ground_truth_tum.txt estimated_tum.txt -va --pose_relation angle_deg 2>&1 | tee rpe_rotational_results.txt 
   ```


**2.用rosbag**

**启动**

```bash
roslaunch test_interface single_drone_avia.launch
roslaunch fast_lio mapping_marsim.launch
 ```
**rosbag记录：**
```bash
rosbag record -O fast_lio_corridor.bag /估计值topic /真值topic    即：
rosbag record -O fast_lio_corridor.bag /Odometry /quad_0/lidar_slam/odom
 ```

**先记录在开运动脚本 （对齐时间戳）**
```bash
  python3 scripts/Lemniscate.py
   ```

# 从同一个bag文件中提取真值轨迹
```bash
evo_traj bag fast_lio_corridor.bag /quad_0/lidar_slam/odom --save_as_tum 
 ```

# 从同一个bag文件中提取估计值轨迹
```bash
evo_traj bag fast_lio_corridor.bag /Odometry --save_as_tum
 ```

后续操作与node完全相同
  
 最后，得到的数据，文件，图片等均在data文件夹
  

  
  
