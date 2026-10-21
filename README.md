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

bash bash/sim_nav_all_start.sh

### 4.2 四个关键设计

**(1) 稳定参考路径 + 进度索引**（最关键的一处改动）

上层行为树每 100 ms 用**当前栅格中心**作起点重规划一次。如果控制器每次都采信这条新路径，前视点会在相邻两个栅格中心之间来回跳（±5 cm ≈ ±10° 方向噪声），喂给一个有滞后的底盘就是持续摆动。

因此控制器把**第一次规划的路径固化为参考路径**，之后的重规划只有在"偏离超过 `ref_dev_tol`（默认 0.60 m，说明真的换了路线）"时才替换；投影时只在进度索引附近（前 6 m / 后 1 m）窗口搜索，既快又不会跳到路径的另一段。

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

**(3) 纯跟踪 + 滞后预测**

取路径前方 `L = clamp(0.30 + 0.15v, 0.30, 0.50)` 处的引导点，方向指向它；
控制点不直接用当前位姿，而是用实测速度前推 `predict_dt = 0.15 s`：

$$
p_{\text{ctrl}} = p + v_{\text{meas}}\cdot \Delta t_{\text{pred}}
$$

这一项补偿掉底盘的速度响应滞后，实测平均跟踪误差从 0.139 m 降到 0.095 m。

**(4) 卡住脱困（兜底）**
命令 >0.5 m/s 而实测 <0.08 m/s 持续 2 s → 沿路径反方向倒车 0.9 s；**一次任务最多 3 次**，防止"蹭墙→倒车→再蹭"的死循环。

最后把 map 系速度旋到车体系：`vx_b = cos(yaw)*ux + sin(yaw)*uy`，`vy_b = -sin(yaw)*ux + cos(yaw)*uy`。
`yaw` 每周期都从 TF 重读（仿真里云台以 1 rad/s 匀速扫描，缓存 yaw 会直接跑飞）。

