# AGT Mapping Framework

面向三维 LiDAR 建图研究与工程验证的 ROS 2 框架。该仓库为博士生与工程团队提供一个可复现、可替换后端、可追溯产物的研究边界：研究可以专注于 LIO、优化、动态过滤和地图表达，而不与底盘控制、Nav2 运行时或巡检业务逻辑耦合。

当前仅完成仓库初始化、架构与迁移规划；**没有迁移、复制或修改任何建图实现源码**。所有未来迁移须先经过 `migration_manifest.yaml` 的人工确认。

## 项目定位

- 提供从 MID360 LiDAR/IMU 采集到三维地图产物的完整建图流水线。
- 为 FAST-LIO2、Batch-LIO、PGO 和 HBA 建立可替换、可评估的后端边界。
- 将地图产物、标定、参数和处理历史固化为可复现的实验记录。
- 为动态目标剔除、语义过滤、可通行性和增量更新等研究提供独立试验位置。

## 当前实现能力

当前能力是已审计的参考基线，而不是本仓库已经迁入的实现：

- MID360 / Livox `CustomMsg` 与 `PointCloud2` 格式适配；
- FAST-LIO2 或 Batch-LIO LiDAR-Inertial Odometry 前端；
- 基于关键帧的 PGO 回环与位姿图优化；
- 基于导出关键帧 patch 与轨迹的 HBA 离线精化；
- `map.pcd`、轨迹、关键帧 patch、元数据及可选导航/重定位派生产物导出。

## 输入与输出

| 输入 | 语义 |
| --- | --- |
| LiDAR 点云 | MID360 `CustomMsg` 或规范化 `sensor_msgs/PointCloud2`，保留原始时间信息。 |
| IMU | `sensor_msgs/Imu`，与 LiDAR 使用一致的时间基准。 |
| 标定 | LiDAR-IMU 外参、frame 约定、时间偏置及传感器 profile。 |
| 可选初值 | 外部姿态、已知地图或离线处理配置。 |

| 输出 | 语义 |
| --- | --- |
| `map.pcd` | 优化后稠密点云地图。 |
| trajectory | 带时间戳的局部与全局优化轨迹。 |
| patches | 关键帧 body-frame 点云及其优化位姿。 |
| `metadata.yaml` | 标定、frame、参数、软件版本、输入与输出校验信息。 |

完整字段和约束见 [docs/interface_contract.md](docs/interface_contract.md)。

## 研究方向

- 动态目标检测、剔除与跨时刻静态置信度建模；
- 基于语义的点云过滤与地图分层表达；
- 由三维点云派生地面、坡度、障碍物与可通行性地图；
- PGO/HBA 参数、鲁棒因子和大范围地图一致性优化；
- 增量建图、地图差分、版本化更新与长期地图维护。

## 与 `agt_navigation_v3` 的关系

`agt_navigation_v3` 是机器人运行时系统，负责定位接入、`map -> odom` TF ownership、Nav2、任务、HMI、底盘和地图审批/激活。本仓库负责产生可审计的建图产物，不发布或控制导航运行时全局 TF，也不拥有地图部署决策。

两者通过版本化地图产物和稳定接口协作：framework 导出已验证的 PCD、轨迹、patch 与 metadata；navigation 系统审核、激活并消费经批准的派生产物。当前参考来源仍位于 `agt_navigation_v3`，不代表代码已经被迁移。
