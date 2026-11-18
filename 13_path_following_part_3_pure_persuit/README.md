# 第 13 章 · Gazebo 路径跟踪

本章在 ROS Noetic 和 Gazebo 中运行纯点跟踪（Pure Pursuit）和 Stanley。以下步骤用于在图形界面中运行一次实验。

## 单次实验

确认 `ros1_car` 容器已启动。在宿主机打开四个终端，**每个终端**先执行：

```bash
sudo docker exec -it ros1_car bash
source /opt/ros/noetic/setup.bash
source /root/car_ws/devel/setup.bash
```

### 终端 1 · 启动 Gazebo

```bash
roslaunch gazebo_ros empty_world.launch
```

等 Gazebo 场景加载后，再执行下一步。保持此终端运行。

### 终端 2 · 生成车辆并加载路径

```bash
roslaunch car_model spawn_car.launch
```

这一步会生成 `smart` 车辆，启动底盘控制节点，并加载和发布参考路径。等车辆出现在场景中后，再启动路径跟踪算法。

### 终端 3 · 选择路径跟踪算法

每次实验在下面两条命令中选一条。两种算法都会向同一个车辆控制话题发送指令。

**纯点跟踪**

```bash
roslaunch pure_persuit pure_persuit.launch
```

**Stanley**

```bash
roslaunch pure_persuit stanley.launch
```

比较两种算法时，先用 `Ctrl+C` 停止当前算法。要从同一初始状态重新开始，先结束当前 Gazebo 和车辆启动进程，再按终端 1–3 的顺序运行下一次试验。

### 终端 4 · 使用 RViz 查看路径

```bash
rviz -d "$(rospack find car_model)/rviz_config/smart.rviz"
```

如果想从车辆起步时就看到参考路径和跟踪过程，可在启动终端 3 之前打开 RViz。Gazebo 和 RViz 的窗口需要容器能够访问宿主机的图形显示。

## 批量实验

批量试验在 `ros1_car` 内运行。脚本会为每次试验启动独立的 ROS master 和无界面 Gazebo，生成车辆并将其复位到参考路线起点。

### 在宿主机准备共用指标模块

容器把 `self-driving-vehicle-101` 挂载到 `/root/project`。轨迹进度和指标计算复用工作区里的两个 Python 模块，启动批量试验前先将它们复制到容器。容器重建或模块更新后再复制一次。

```bash
cd /home/le/workspace/study/basic_course/UGV
sudo docker start ros1_car
sudo docker exec ros1_car mkdir -p /tmp/ugv-gazebo-metrics
sudo docker cp carla_town_loop_tracking/progress.py ros1_car:/tmp/ugv-gazebo-metrics/
sudo docker cp carla_town_loop_tracking/evaluate.py ros1_car:/tmp/ugv-gazebo-metrics/
sudo docker exec -it ros1_car bash
```

### 在容器内启动批量试验

```bash
source /opt/ros/noetic/setup.bash
source /root/car_ws/devel/setup.bash
cd /root/project/13_path_following_part_3_pure_persuit

# 快速试跑，两种算法各一次
python3 -B gazebo_tracking/sweep.py --in-container --speeds 10 --repeats 1

# 正式批次，默认 S 形路线，两种算法在 5、10、15 km/h 各重复 3 次
python3 -B gazebo_tracking/sweep.py --in-container
```

用 `--speeds` 修改整数速度档，单位为 km/h。用 `--controllers pure_pursuit stanley` 选择算法。可重复传入 `--route /容器内路径/route.csv` 添加路线。路线 CSV 为无表头的 `x,y,yaw`，单位依次为米、米、弧度，路线需长于 20 米。

只跑一次并保存数据时，可在同一个容器终端执行：

```bash
python3 -B gazebo_tracking/run.py --in-container --controller stanley --speed 10
```

### 在宿主机查看结果和生成图表

批量结果通过挂载写入本章的 `gazebo_tracking/results/matrix_日期_时间/`。其中有参考路线副本、单次轨迹和日志，以及与 CARLA 同字段的 `summary.csv`。位置为车辆后轴位置，速度为实测车速，转向为控制器指令，采样标称周期为 0.05 仿真秒。跟踪误差、实测均速和转向变化率统计剔除路线首尾各 10 米。

Gazebo 空地上的 `outside_lane` 表示后轴曾偏离参考路线超过 `--half-width`，默认 1.75 米。完成判据为走到路线末端，并在距终点 1 米内停稳至少 1 秒。跟踪未达标的试次也会写入汇总。

回到宿主机，将 `batch` 替换为实际的批次目录。容器以 root 写入批次，先将该目录归还给宿主机当前用户，以便生成图表。

```bash
cd /home/le/workspace/study/basic_course/UGV/self-driving-vehicle-101/13_path_following_part_3_pure_persuit
batch=gazebo_tracking/results/matrix_20261117_120645
sudo chown -R "$(id -u):$(id -g)" "$batch"
conda run --no-capture-output -n carla python -B ../../carla_town_loop_tracking/analysis/generate_results.py \
  "$batch" --output-dir "$batch/analysis"
```

同一张画布比较参考路径和两种算法的实际路线，在宿主机的本章目录执行：

```bash
conda run --no-capture-output -n carla python -B gazebo_tracking/plot_paths.py "$batch"
```

输出到该批次的 `analysis/`。每档目标速度一列，上排叠加完整参考路径与两种算法各自第 1 次试验的实际轨迹，下排放大同一次试验中的同一弯道。可用 `--repeat 2` 或 `--repeat 3` 选择其他试次。不同速度使用相同的坐标范围，横纵坐标等比例，便于比较实际几何偏差。放大区域按参考路径的最大曲率位置选择。实际路线位置为车辆后轴位置。
