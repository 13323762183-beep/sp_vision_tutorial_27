# 27算法组导航方向招新大作业


在给定的迷宫地图上完成「一次点击导航」全流程：
**RViz 点一个终点 → 上层行为树触发导航 → A\* 规划全局路径 → 本仓库实现的控制器插件跟踪路径 → 小车平稳到达终点。**

1. **实现路径跟踪控制器插件** —— `sp_controller_server/plugins/pid_controller.cpp`
   （`configure` / `setPlan` / `computeVelocityCommands` 三个接口）；
2. **补全运行所需参数** —— `sp_nav_bringup/config/nav_params.yaml`
   （ESDF / 规划器 / 控制器三组参数原本全为空，直接运行会因 `[参数缺失]` 抛异常退出）。

## 1. 系统架构与数据流

```
                     RViz "2D Goal Pose"
                             │ /goal_pose
                             ▼
   ┌──────────────────────────────────────────┐
   │ sp_decision  上层行为树 click_nav.xml     │
   └──────────────────┬───────────────────────┘
                      │ NavigateToPose (Action)
                      ▼
   ┌──────────────────────────────────────────┐
   │ sp_nav_bt  下层行为树                     │  20Hz 取位姿，10Hz 重规划，
   │ default_nav_with_fallback.xml             │  剩距 <0.3m 判定到达
   └───────┬───────────────────────┬──────────┘
           │ make_plan (srv)        │ set_control_enable (srv)
           ▼                        ▼
 ┌───────────────────┐      ┌────────────────────────────┐
 │ sp_global_planner │      │ sp_controller_server       │
 │ A* → /global_path │─────▶│ 加载 PidController 插件    │
 └─────────▲─────────┘      │ 50Hz 计算 cmd_vel          │
           │ /global_costmap │                            │
           │ /local_costmap  └──────────┬─────────────────┘
 ┌─────────┴─────────┐                 │ /sentry/cmd_vel
 │ sp_map_server     │      ┌──────────▼─────────────────┐
 │ ESDF 代价地图     │      │ sp_nav_sim 拖拽仿真        │
 └───────────────────┘      │ /Odometry + TF             │
                            └────────────────────────────┘
```

* TF：`map --(静态)--> lidar_odom --(动态)--> base_link`
* 控制器输出 `TwistStamped`（`base_link` 系），`controller_server` 取 `twist` 发布到 `/sentry/cmd_vel`。
  全向底盘只吃 `linear.x / linear.y`，故 `angular.z` 恒为 0。

## 2. 环境与编译

```bash
# Ubuntu 22.04 + ROS 2 Humble
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  ros-humble-pluginlib ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-nav-msgs ros-humble-std-srvs ros-humble-rviz2 \
  ros-humble-behaviortree-cpp-v3 qtbase5-dev libopencv-dev libyaml-cpp-dev
pip3 install --user pygame numpy Pillow

cd <仓库根目录>
colcon build --symlink-install && source install/setup.bash
```

## 3. 运行

```bash
# 终端 1：静态 TF（云台相对车体）
ros2 run tf2_ros static_transform_publisher 0 0.15 0 0 0 0 base_link livox_frame
# 终端 2：代价地图 + 规划 + 控制器 + 决策 + 行为树 + RViz
ros2 launch sp_nav_bringup sp_nav.launch.py
# 终端 3：仿真器（等 3 秒再起）
ros2 launch sp_nav_sim sim_robot.launch.py
```

或 `bash bash/sim_nav_all_start.sh` 一次拉起。

**评测操作**：RViz 工具栏选 `2D Goal Pose`，点击终点 **(14.1, 14.1)**（只点一次）。

## 4. 控制器实现（核心）

### 4.1 先把底盘能力量出来

调参之前先给 `/sentry/cmd_vel` 打阶跃、读 `/Odometry`，把底盘的物理边界量出来：

| 指标 | 实测值 | 对控制的影响 |
|---|---|---|
| 稳态可达线速度 | ≈1.38 m/s | 轮速上限 2.0 被“小陀螺”自旋的 ω×0.2 占掉一块，指令给再大也没用 |
| 速度矢量转向能力 | p90 ≈2.8 rad/s | 指令方向转得比它快，底盘追不上 → 画龙 |
| 速度响应滞后 | ≈0.1–0.15 s | 纯跟踪会因此产生极限环摆动 |
| 车体自旋包络 | 半对角线 0.354 m | 离墙必须 >0.35 m，否则边转边蹭 |

这些数字直接决定了后面的参数取值范围。

### 4.2 四个关键设计

**(1) 稳定参考路径 + 进度索引**（最关键的一处改动）

上层行为树每 100 ms 用**当前栅格中心**作起点重规划一次。如果控制器每次都采信这条新路径，前视点会在相邻两个栅格中心之间来回跳（±5 cm ≈ ±10° 方向噪声），喂给一个有滞后的底盘就是持续摆动。

因此控制器把**第一次规划的路径固化为参考路径**，之后的重规划只有在“偏离超过 `ref_dev_tol`（默认 0.60 m，说明真的换了路线）”时才替换；投影时只在进度索引附近（前 6 m / 后 1 m）窗口搜索，既快又不会跳到路径的另一段。

**(2) 曲率限速 + 刹车反传的速度剖面**

$$
\kappa_i=\frac{|\Delta\theta_i|}{\tfrac12(l_{i-1}+l_i)},\qquad
v^{\text{lat}}_i=\min\left(v_{\max},\ \sqrt{a_{\text{lat}}/\kappa_i}\right)
$$

$$
v_i=\min\left(v^{\text{lat}}_i,\ \sqrt{v_{i+1}^{2}+2a_{\text{dec}}\Delta s_i}\right),\qquad v_{\text{end}}=0
$$

弯道按 `a_lat` 限速，再用刹车约束从终点反向传播，保证每一点都来得及减速，末端自然收敛停车。
路径先做 2 次 `[1/4, 1/2, 1/4]` 平滑，把栅格的直角弯圆角化。

**(3) 全向解耦跟踪：横向误差直接变成平移速度**

这是解决“S 型画龙”的核心。把 map 系速度拆成**沿路径切向**和**垂直路径法向**两路：

$$
v_n=-\left(k_p e_{\text{lat}}+k_d\dot e_{\text{lat}}\right),\qquad
v_t=\sqrt{\max\left(0,\ v^2-v_n^2\right)}
$$

$$
\mathbf{v}=v_t\,\hat{\mathbf{t}}+v_n\,\hat{\mathbf{n}}
$$

`e_lat` 是投影点到当前位姿的有符号横向偏差，`ė_lat` 取实测速度在法向上的投影（天然的速度阻尼）。切向用勾股补偿，保证横向纠偏不拖慢沿路径的前进速度，合成速度始终等于规划的 `v(s)`。

**关键点**：全向底盘本来就能横着走，横向偏差就该横向修。之前把横向误差折算成航向角增量（`dir += -k·e/v`）是差速/阿克曼车的做法，放到全向底盘上必然“朝左前方冲 → 冲过头 → 朝右前方冲”，轨迹就是 S 型。

**(4) 卡住脱困（兜底）**

命令 >0.5 m/s 而实测 <0.08 m/s 持续 2 s → 沿路径反方向倒车 0.9 s；**一次任务最多 3 次**，防止“蹭墙→倒车→再蹭”的死循环。

最后把 map 系速度旋到车体系：`vx_b = cos(yaw)*ux + sin(yaw)*uy`，`vy_b = -sin(yaw)*ux + cos(yaw)*uy`。
`yaw` 每周期都从 TF 重读（仿真里云台以 1 rad/s 匀速扫描，缓存 yaw 会直接跑飞）。

### 4.3 为什么原来的跟踪是 S 型

同一张迷宫、同一条 A\* 路径、同一个一阶速度滞后车模型（τ=0.18 s）下闭环复现控制律：

| 做法 | 横向误差 RMS | 最大偏差 | 摆动次数（误差符号翻转） |
|---|---|---|---|
| 横向误差折算成航向角（原） | 9.39 cm | 25.7 cm | 15 |
| 同上，再叠加横向 PD（k=0.8 / 0.6） | 8.10 cm | 21.1 cm | 13 |
| **横向误差直接给法向速度（现）** | **0.55 cm** | **1.3 cm** | **0** |

只调参数救不回来（横向 PD 加上去也只是 15 次→13 次），因为问题不在增益大小，而在**控制量用错了通道**——横向偏差被送进了方向角，而方向角只能靠“斜着冲”去抵消横向偏差。

`decouple_tracking: false` 可以一键退回原做法做对比。

### 4.4 参数一览

| 参数 | 值 | 含义 |
|---|---|---|
| `max_linear_vel` | 1.35 m/s | 实测底盘上限 1.38，不贪 |
| `lat_acc` | 1.8 m/s² | 过弯向心加速度（过弯速度 ≈√(a_lat·R)） |
| `dec_acc` | 1.2 m/s² | 速度剖面刹车减速度 |
| `lookahead_min/max/gain` | 0.30 / 0.50 / 0.15 | 前视距离 `min + gain·v` |
| `predict_dt` | 0.15 s | 滞后预测时间 |
| `ref_dev_tol` | 0.60 m | 换参考路径的偏离阈值 |
| `decouple_tracking` | true | 解耦跟踪开关（false=退回纯跟踪方向角） |
| `dec_kp` / `dec_kd` | 2.0 / 0.35 | 法向速度的横向误差 PD |
| `dec_vmax` | 0.60 m/s | 法向纠偏速度上限 |
| `goal_tolerance` | 0.05 m | 终点停车阈值 |
