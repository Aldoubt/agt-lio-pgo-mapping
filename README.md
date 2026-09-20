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
