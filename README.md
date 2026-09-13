# AGT LIO–PGO Mapping

ROS 2 Humble 的三维 LiDAR 建图基线：**Livox MID-360 → FAST-LIO2 → PGO → 可校验 PCD 地图包**。

它面向建图研究协作，不包含 Nav2、定位运行时、HMI、RTK 或底盘控制。FAST-LIO2、PGO、HBA 和 Batch-LIO 保持为锁定版本的外部依赖；本仓库只实现稳定的传感器、前后端和地图产物接口。

> MID-360 是 Livox 雷达。当前基线已通过 `bunker_mid360_mapping_20260901_205036` 的真实 rosbag 回归。

## 一键开始

前提：Ubuntu 22.04、ROS 2 Humble、网络连接，以及可使用 `sudo` 安装系统依赖。

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Aldoubt/agt-lio-pgo-mapping.git
cd agt-lio-pgo-mapping
./scripts/bootstrap.sh
```

脚本会锁定并获取 FAST-LIO2/PGO、Livox driver 和 Batch-LIO，安装 Humble 依赖并构建所需 package。

## 运行建图

给定 MID360 rosbag 目录：

```bash
cd ~/ros2_ws/src/agt-lio-pgo-mapping
./scripts/run_mid360_mapping.sh /path/to/mid360_mapping_bag
```

该命令会自动启动 RViz、播放 rosbag，并在播放结束后触发 PGO 与地图导出。RViz 中展示的是**实时 LIO**点云与轨迹；最终可交付地图以导出的 `map_package` 为准。

检查导出结果：

```bash
./scripts/verify_map_artifact.sh ~/ros2_ws/output/<run_name>
```

合格产物目录结构：

```text
map_package/
├── map.pcd
├── poses.txt
├── poses_timed.txt
├── patches/
├── calibration.yaml
├── metadata.yaml
├── manifest.yaml
└── checksums.sha256
```

仅当 `metadata.yaml` 中存在 `backend: PGO` 和 `optimized: true`，且校验脚本成功时，才应交付地图。

## 技术边界

```text
MID-360 CustomMsg + IMU
          ↓
MID360 adapter（校验扫描并保留时间/强度）
          ↓
FAST-LIO2 frontend
          ↓
Keyframes + PGO
          ↓
Optimized PCD map artifact
```

详细接口、地图格式、架构和研究扩展点见 [`docs/`](docs/)，交付验收记录见 [docs/delivery_acceptance.md](docs/delivery_acceptance.md)。本仓库产生版本化地图资产；`agt_navigation_v3` 负责审核并在机器人运行时使用这些资产。
