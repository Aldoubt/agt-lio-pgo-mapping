# AGT LIO–PGO Mapping

ROS 2 Humble 的三维 LiDAR 建图基线：**Livox MID-360 → FAST-LIO2 → PGO → 可校验 PCD 地图包**。

它面向建图研究协作，不包含 Nav2、定位运行时、HMI、RTK 或底盘控制。FAST-LIO2、PGO、HBA 和 Batch-LIO 保持为锁定版本的外部依赖；本仓库只实现稳定的传感器、前后端和地图产物接口。

> MID-360 是 Livox 雷达。当前 **0.2.0 编排升级**已在共享目录对应的 Ubuntu 22.04 / ROS 2 Humble 宿主机完成 82 项自动化测试、完整基准录包回放及操作场景验证。MCP 的 Ubuntu 24.04 容器不是 ROS 执行环境。具体版本覆盖、测试安装层及剩余验收边界见 [宿主机测试报告](docs/mcp-host-validation.md)。

## 一键开始

前提：Ubuntu 22.04、ROS 2 Humble、网络连接，以及可使用 `sudo` 安装系统依赖。

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Aldoubt/agt-lio-pgo-mapping.git
cd agt-lio-pgo-mapping
./scripts/bootstrap.sh
```

脚本会锁定并获取 FAST-LIO2/PGO、Livox driver 和 Batch-LIO，安装 Humble 依赖并构建所需 package。

已有工作区的全量/增量重编译、自动化测试、入口冒烟测试和现场验收步骤见 [编译与测试流程](docs/build_and_test.md)。

## 运行建图

给定 MID360 rosbag 目录：

```bash
cd ~/ros2_ws/src/agt-lio-pgo-mapping
./scripts/run_mid360_mapping.sh /path/to/mid360_mapping_bag
```

新入口会预检 metadata、存储分片、非空 CustomMsg/IMU 话题及输出目录，然后等待处理链就绪再回放。仅正常结束才请求 PGO 导出，只有完整地图通过校验才报告成功，并默认自动关闭本次 launch。回放失败/取消不会误触发导出，旧输出不会被覆盖。

常用操作：

```bash
./scripts/run_mid360_mapping.sh /path/to/bag --dry-run             # 不启动 ROS、不创建输出
./scripts/run_mid360_mapping.sh /path/to/bag --no-rviz --rate 1.0 # 无界面自动完成
./scripts/run_mid360_mapping.sh /path/to/bag --start-paused       # 准备好后人工恢复
./scripts/run_mid360_mapping.sh /path/to/bag --keep-open          # 完成后保留可视化
./scripts/run_mid360_mapping.sh --help                           # 无需 ROS
```

`--dry-run` 需要 Python 3 + PyYAML。运行前请在宿主机 Humble 环境重建新版 `agt_mapping_bringup` / `agt_mapping_artifacts` / `agt_mapping_exporter`，见 [构建与完整操作指南](docs/runtime_setup.md)。wrapper 默认使用本机回环和独立 ROS domain 89；并发任务使用不同 `--domain-id`，不要与实机运行时混用。

有图形环境时默认打开 RViz，无显示时自动无界面运行。RViz 中展示的是**实时 LIO**点云与轨迹，不是最终 PGO 地图；最终可交付地图以导出的 `map_package` 为准。运行阶段和失败原因记录到输出目录的 `session.json`。

检查导出结果：

```bash
./scripts/verify_map_artifact.sh ~/ros2_ws/experiments/artifacts/output/<run_name>
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

仅当 `metadata.yaml` 中存在 `backend: PGO`、`backend_status.optimized: true`，并且非空地图、必需文件、完整 checksum 清单及 SHA-256 全部通过校验时，才应交付地图。导出服务返回成功仅表示受理请求，不代表写入或校验完成。

### 单 MID360 实机建图（0.3.0）

```bash
./scripts/run_mid360_live_mapping.sh [OUTPUT] --no-rviz          # 启动 Livox 驱动 + LIO/PGO，同步录制 raw_bag/
ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 \
  ros2 service call /mapping/session/finish std_srvs/srv/Trigger "{}"   # 或 touch <OUTPUT>/STOP_MAPPING
```

等价于 `run_mid360_mapping.sh --live`；支持 `--livox-config`、`--duration`、`--sensor-stall-seconds`、`--dry-run`。原始 `/livox/lidar` + `/livox/imu` 始终录制到 `<OUTPUT>/raw_bag`，可用回放模式复现。

### 建图后自动转二维图并人工确认

```bash
./scripts/run_mid360_mapping_review.sh \
  /home/yangxuan/ros2_ws/experiments/data/rosbag/bunker_mid360_mapping_20260901_205036 \
  /home/yangxuan/ros2_ws/experiments/artifacts/output/bunker_mid360_review_$(date +%Y%m%d_%H%M%S) \
  --lidar-topic /agt/sensors/lidar/custom \
  --imu-topic /agt/sensors/imu/data
```

该入口先正常回放建图；最终 PGO 地图通过校验后，自动调用本仓库的
`agt_pcd2grid_exporter` 生成 PGM/YAML，再以轻量二维模式打开 Map Studio。
此模式不会把大 PCD 加载到 OpenGL，编辑完成前不会产生“已确认”地图。使用
`Erase rect`、`Obstacle line`、三种 polygon 或 `Forbidden zone` 修改后，点击
`Confirm & Save 2D Map`，结果写入：

```text
<OUTPUT>/map_review/
├── base/                   # 自动转换的原始二维图
│   ├── map.pgm
│   └── map.yaml
└── confirmed/              # 仅人工确认后创建/更新
    ├── map.pgm
    ├── map.yaml
    ├── map_refinement.yaml
    ├── keepout_zones.yaml
    ├── metadata.yaml
    └── review_status.yaml  # status: confirmed
```

已经完成建图时无需再回放录包，可直接审核已有输出：

```bash
./scripts/review_mapping_output.sh /path/to/completed_mapping_output
```

这条 PCD→PGM→人工编辑→确认链路只依赖 `agt_mapping_framework` 内的包，
不依赖、不检测也不调用 `agt_navigation_v3`。

二维转换默认还会使用建图包中的 `patches/*.pcd` 和 `poses_timed.txt` 做静态持久性
过滤：20 cm 三维体素需要被至少两个、且相隔至少两个关键帧重复观测，随后按局部
地面相对高度提取障碍、删除小孤立区域并做一格闭运算。这样可过滤跟车人员等只在
少量连续帧出现的拖影，同时保留墙、路沿和立柱。原始 `map.pcd` 与建图包不会被修改。
长时间原地不动的人仍可能被当成静态物体，需在二维编辑器中删除；若现场仍频繁出现，
再考虑在建图前端增加语义动态目标过滤，而不是直接改变 SLAM 主链。

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

详细接口、地图格式、架构和研究扩展点见 [`docs/`](docs/)，交付验收记录见 [docs/delivery_acceptance.md](docs/delivery_acceptance.md)。本仓库独立完成建图、PCD→PGM、二维编辑和人工确认；确认后的地图可由其他运行时按需使用。
