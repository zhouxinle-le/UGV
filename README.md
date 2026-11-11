# Town04 两条环线的单圈路径跟踪

每次实验行驶一圈。路线 1 在原起点旁边的右车道结束，路线 2 回到自己的起点。

| `--case` | 行驶方式 |
| --- | --- |
| `forward` | **逆时针大环，2096.03 m**。从道路 23 左车道 −2（`s=240 m`）出发，保留 1368 号路口转弯辅助线修正并接入道路 40 的车道 4。沿主路到道路 50，在虚线段依次变到车道 5、6。回程经 1159 号路口的连接道 1162，进入道路 23 的右车道 −3，沿右车道行驶至与起点平齐的 `s=240 m`。 |
| `clockwise` | **顺时针小环，463.37 m**。从道路 8 的车道 1（`s=22 m`）出发，沿道路 51、16、52 回到道路 8，四个路口依次为 1197、148、916、1103。全程沿地图规定的车道方向行驶。 |

路线从 `Town04_Opt` 的 OpenDRIVE 数据生成。`run.py` 将参考路线写入 `routes/Town04_Opt_forward.csv` 或 `routes/Town04_Opt_clockwise.csv`，结果也使用对应的路线标识。`--use-current-map` 可以复用已经加载的 Town04 或 Town04_Opt。

试验复用 `ros1_car` 内的 Pure Pursuit 和 Stanley 控制器。程序在容器内临时启动本项目的 ROS 路点节点，按已行驶的路线里程发布前视路点，在一圈末尾减速停车。仿真采用 0.05 s 同步步长。

## 启动

保留 `ros1_car` 和其中的 rosbridge，默认 WebSocket 端口为 9090。需要启动 rosbridge 时，在容器内执行：

```bash
source /opt/ros/noetic/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch address:=127.0.0.1 port:=9090
```

CARLA 已在 2012 端口运行时可直接复用。需要启动时，先检查 2012–2014 是否空闲：

```bash
ss -ltnp '( sport = :2012 or sport = :2013 or sport = :2014 )'
cd /home/le/apps/carla
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
  __NV_PRIME_RENDER_OFFLOAD=1 __VK_LAYER_NV_optimus=NVIDIA_only \
  ./CarlaUE4.sh -quality-level=Low -carla-rpc-port=2012
```

仅做批量试验时，可在 CARLA 启动命令末尾加 `-RenderOffScreen`。另开终端验证 RPC：

```bash
conda run -n carla python /home/le/apps/carla/PythonAPI/util/test_connection.py \
  --port 2012 --timeout 30
```

## 单次和批量实验

```bash
cd /home/le/workspace/study/basic_course/UGV/carla_town_loop_tracking

conda run --no-capture-output -n carla python run.py \
  --carla-port 2012 --case forward --controller pure_pursuit --speed 15

conda run --no-capture-output -n carla python run.py \
  --carla-port 2012 --case clockwise --controller stanley --speed 15

conda run --no-capture-output -n carla python sweep.py --carla-port 2012 --repeats 3
```

`--speed` 的单位是 km/h。单跑加 `--no-rendering` 可加速，批量脚本自动启用此选项。带图形界面时默认使用随车视角。两条路线始终各跑一圈。

批量脚本依次运行逆时针大环和顺时针小环，每条路线使用 5/10/15 km/h 三档速度及两种控制器。每个组合重复 3 次，共 36 次。结果目录位于 `results/`，包括每次试验的轨迹 CSV 和日志，以及汇总表 `summary.csv`。

轨迹记录时间、后轴坐标、速度、转向指令、沿路线进度 `progress_m` 和跟踪距离误差 `cross_track_m`。RMSE、最大误差、实测平均速度和转角变化率 RMS 从距路线两端各 10 m 的中段计算。`outside_lane` 标记后轴偏离参考路径超过当地车道半宽。`completed` 要求车走到本圈末尾，并在距终点 1 m 内连续停稳至少 1 s。
