# Mobile-Robot-Localization-Perception-Framework
## 🚀 项目进度 (Roadmap)
- [x] **Phase 1: Multi-Sensor Fusion Localization (已完成)**
  - 底层通信：纯 Python 实现 4 天线 UWB 与维特 IMU 的串口二进制解析（含防粘包机制）。
  - 数据处理：滑动窗口均值滤波、死区防抖处理。
  - 融合引擎：基于 ROS 2 `robot_localization` 的 EKF 扩展卡尔曼滤波紧耦合，输出全局 Odometry 与 TF 树。
    <p align="center">
    <img src="<01-ROS2_UWB_IMU_Fusion/record/Screenshot 2026-10-26 003450.png>" width="80%" alt="RViz Fusion Demo">
    <br>
    <em>图 1: RViz 中的 EKF 多源融合轨迹可视化</em>
  </p>
- [ ] **Phase 2: Visual SLAM & Navigation (进行中)**
  - 引入深度相机，运行 VIO 算法实现视觉惯性里程计。
  - 结合 Nav2 导航栈，实现复杂未知环境下的自主建图与路径规划。
- [ ] **Phase 3: Embodied AI / VLA Integration (规划中)**
  - 接入视觉伺服抓取系统。
  - 探索自然语言指令下发到底层移动与操作任务的闭环。
