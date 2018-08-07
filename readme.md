Calibration Package
===========================================

# 简介
该项目为以地面为基准的激光雷达标定工具，用于对激光雷达初始安装姿态的校准和车辆行进过程中抖动的校准．

## 初始校准
由于雷达在安装固定后存在一定的安装误差，为了提高地面点过滤的准确性，我们需要对该机械误差进行校准，所用到的算法为`（SAC_RANSAC）随机采样一致性`．具体是通过地面作为基准，估算激光雷达的姿态．可以通过实时点云或者点云数据包，获取激光雷达的姿态．

### 数据要求与标定步骤
```
*　尽量保证车辆所在的以至少１５米为半径（若雷达安装高度超过１．８米，该数值应当更大）的区域中，有尽可能少的障碍物．
*　所在区域的地面尽可能保持平整，没有减速带，草坪，坑哇，积水等．
*　若雷达安装时已经存在较大的倾斜，应考虑对遮挡区域的点云进行过滤处理，以免造成标定不准．
*　可以使用ｒｖｉｚ实时观测所获取的地面信息，如果存在一定偏差和不稳定，可对程序算法中的部分参数进行修改，以达到要求．
```

### 程序运行与参数说明
```
roslaunch calibration init_calibration.launch
```
- `points_cloud_topic` 输入点云和话题名称． Default `pandar_points`

- `output_file_path`　生成ｙａｍｌ的文件路径． Default `/home/ding/Documents/SLAM/src/calibration/param/lidar_init_calibration.yaml`

### 默认输出话题
地面点云.

## 实时校准
车辆在行进过程中，车辆本身会有晃动，大幅度的晃动会影响点云的处理，所以为了获取点云较为理想的过滤效果，需要实时对每一帧点云进行处理．处理的方法与初始标定类似，不同的是我们会根据标定结果，对当前点云进行旋转平移和滤波处理．

### 实时校准介绍
```
*　为了保证运算速度，我们对点云进行了ｖｏｘｅｌ＿ｇｒｉｄ处理，精度为１０ｃｍ．同时选取的点云大小为半径４０米的圆．（可根据需要修改参数）
*　当前只输出地面点云和非地面点云，点云已经经过实时矫正，可能据需要修改部分代码来获取，原始点云，姿态或是其他信息．
```

### 程序运行和参数说明
```
roslaunch calibration realtime_calibration.launch
```
- `points_cloud_topic` 输入点云和话题名称． Default `pandar_points`

- `input_file_path`　读取ｙａｍｌ的文件路径． Default `/home/ding/Documents/SLAM/src/calibration/param/lidar_init_calibration.yaml`

- `ground_topic` 地面点云话题名称．Default `ground`

- `no_ground_topic`　非地面点云话题名称．Default `no_ground`

### 默认输出话题
- 地面点云
- 非地面点云
