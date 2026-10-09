# AGT LIO–PGO Mapping

## Product integration V1 — staged for cloud review

当前独立分支提供可选传感器输入、FAST-LIVO2 默认建图前端以及独立手动地图处理的显式配置；未验收的 Point-LIO、LIO-SAM、动态清理/地形/路线后端不会自动启动。详细见 [产品边界与命令](docs/product_mapping_architecture_v1.md) 和 [集成审计](docs/audits/mapping_product_integration_20261009.md)。

```bash
bash scripts/run_mapping.sh --capabilities
bash scripts/run_mapping.sh /path/to/bag --dry-run --product-config bringup/agt_mapping_bringup/config/product_pipeline.yaml
```

这次仅迁入 Operator Workflow 的离线 Study CLI 与独立渲染抽样助手，原生 MapStudio GUI 尚未完成分支融合和真实界面验收。`main` 与原分支均未改变。

## MapStudio Relocalization MVP — SOFTWARE PARTIAL

Implemented: immutable keyframe blocks and index, evidence contracts, sparse offline GLOBAL/Top-K/GICP queries, candidate inspection and query/candidate overlays. The KF590 green-house run recorded a same-session `FALSE_ACCEPT` diagnostic. Reviewed row/headland freeze, row-middle/end/headland query coverage, a representative 20–50 query set, interactive close/reopen acceptance, independent localization ground truth, and the scientific conclusion remain incomplete. This is not production-ready.

ROS 2 Humble 的三维 LiDAR 建图框架：用户指定默认 FAST-LIVO2 LIO-only（LiDAR+IMU，关闭视觉），支持明确选择其它已审核 source；关键帧分块不要求 PGO 优化。本项目工作流程不使用 FAST-LIO2，已有 legacy 代码仅保留历史兼容参照。

**当前操作员建图入口已迁移为 FAST-LIVO2 LIO-only。** offline/live CLI 与 `mapping_v0` 使用 `fast_livo2_lio`、关闭视觉且不启动 PGO；运行时需加载 profile 指向的 FAST-LIVO2 backend overlay。`green-house` 全量回放和 source package 校验已通过；这只证明采集、导出和格式完整性，不证明地图几何质量或重定位正确性。PCD→PGM、交互编辑和 cold-start / mid-route-loss 配对诊断仍未由本轮验证。

它面向多源建图与离线地图研究；导航运行时、在线控制和底盘执行属于导航/平台仓库。FAST-LIO2、PGO、HBA 和 Batch-LIO 保持为锁定版本的外部依赖。

## 当前需求与 AI 工作入口

目标流程是多源输入 → 复用关键帧方法分块 → 离线重定位歧义分析/可视化/环境改造评估 → 导航地图与离线路线编辑 → 既有合同发布。MapStudio 与外部 HMI 是 authoring client；通用 HMI API、分块资产化和 Route authoring 仍为待实现需求。

总体对照与阶段验收见 [需求计划](docs/map_task_asset_authoring_alignment_plan.md)，后续 AI 从 [最小任务阅读集](docs/ai_task_entrypoint.md) 进入；仓库规则见 [AGENTS.md](AGENTS.md)。[A0 验收](docs/contracts/a0_authoring_contract_acceptance.md) 当前为 PARTIAL，需求更新不自动启动 A1–A7。下文保留既有运行说明，历史测试数字不能替代当前验收。

MapStudio Annotation 和 Map Edit 的当前 GUI 操作、Before/After 截图与控件清点见 [UX Cleanup V1](docs/research/mapstudio_ux_cleanup_v1.md)。

> MID-360 是 Livox 雷达。当前 **0.2.0 编排升级**已在共享目录对应的 Ubuntu 22.04 / ROS 2 Humble 宿主机完成 82 项自动化测试、完整基准录包回放及操作场景验证。MCP 的 Ubuntu 24.04 容器不是 ROS 执行环境。具体版本覆盖、测试安装层及剩余验收边界见 [宿主机测试报告](docs/mcp-host-validation.md)。

## 可复用的温室拓扑标注与 Top-K 分析

对新场景的 `map_package` 手工标注真实行中心线、地头区域和 scene markers：

```bash
./scripts/annotate_map_topology.sh /path/to/new/map_package /path/to/new/annotation
```

支持编辑、撤销/重做、YAML/GeoJSON/关键帧标签导出及冻结校验。冻结后可接原生 GLOBAL Top-K trace 和论文表图导出，见 [完整流水线](docs/paper/GREENHOUSE_TOPOLOGY_TOPK_PIPELINE.md) 与 [标注工具](tools/agt_greenhouse_annotation/README.md)。

## 一键开始

前提：Ubuntu 22.04、ROS 2 Humble、网络连接，以及可使用 `sudo` 安装系统依赖。

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Aldoubt/agt-lio-pgo-mapping.git
cd agt-lio-pgo-mapping
./scripts/bootstrap.sh
```

脚本会锁定并获取历史 FAST-LIO2/PGO、Livox driver 和 Batch-LIO 依赖，安装 Humble 依赖并构建所需 package。操作员默认 workflow 使用 FAST-LIVO2 LIO-only，不启动 FAST-LIO2 或 PGO。

已有工作区的全量/增量重编译、自动化测试、入口冒烟测试和现场验收步骤见 [编译与测试流程](docs/build_and_test.md)。

## 运行建图

给定 MID360 rosbag 目录：

```bash
cd ~/ros2_ws/src/agt-lio-pgo-mapping
./scripts/run_mid360_mapping.sh /path/to/mid360_mapping_bag
```

新入口会预检 metadata、存储分片、非空 CustomMsg/IMU 话题及输出目录，然后等待 FAST-LIVO2、frontend adapter 和 exporter 就绪再回放。仅正常结束才请求 paired source-package 导出；现有 `verify_frontend_map_package` 验证通过后才报告成功，并默认自动关闭本次 launch。回放失败/取消不会误触发导出，旧输出不会被覆盖。

常用操作：

```bash
./scripts/run_mid360_mapping.sh /path/to/bag --dry-run             # 不启动 ROS、不创建输出
./scripts/run_mid360_mapping.sh /path/to/bag --no-rviz --rate 1.0 # 无界面自动完成
./scripts/run_mid360_mapping.sh /path/to/bag --start-paused       # 准备好后人工恢复
./scripts/run_mid360_mapping.sh /path/to/bag --keep-open          # 完成后保留可视化
./scripts/run_mid360_mapping.sh --help                           # 无需 ROS
```

`--dry-run` 需要 Python 3 + PyYAML。运行前请在宿主机 Humble 环境重建新版 `agt_mapping_bringup` / `agt_mapping_artifacts` / `agt_mapping_exporter`，见 [构建与完整操作指南](docs/runtime_setup.md)。wrapper 默认使用本机回环和独立 ROS domain 89；并发任务使用不同 `--domain-id`，不要与实机运行时混用。

有图形环境时默认打开 RViz，无显示时自动无界面运行。RViz 展示 FAST-LIVO2 LIO 点云与轨迹；导出的 `map_package` 是同一 frontend session 的 patch+pose source reference，不是独立 ground truth。运行阶段和失败原因记录到输出目录的 `session.json`。

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

导出服务返回成功仅表示受理请求，不代表写入或校验完成。操作员 session 只在 `verify_frontend_map_package` 通过后标记完成；该 validator 不证明地图质量或绝对定位准确性。

### 单 MID360 实机建图（0.3.0）

```bash
./scripts/run_mid360_live_mapping.sh [OUTPUT] --no-rviz          # 启动 Livox 驱动 + FAST-LIVO2 LIO-only，同步录制 raw_bag/
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

该入口先正常回放建图；最终 source map package 通过校验后，自动调用本仓库的
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

## 多 LIO Backend profiles

本分支增加了显式 backend 选择和 backend-independent `map_package`：

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/agt_navigation_v2/install/setup.bash
source /home/yangxuan/lio_benchmark_algorithms/point_lio_ws/install/setup.bash
source /home/yangxuan/lio_benchmark_algorithms/lio_sam_ws/install/setup.bash
source /home/yangxuan/ros2_ws/experiments/multi_lio_backend_20261003/install/setup.bash

ros2 launch agt_mapping_bringup mapping.launch.py \
  bag_path:=/home/yangxuan/rosbags/green-house \
  output_dir:=/home/yangxuan/ros2_ws/experiments/multi_lio_backend_20261003/runs/lio_sam_noloop \
  mapping_backend:=lio_sam_noloop
```

低层 `mapping.launch.py` 可显式选择 `lio_sam_noloop`、`point_lio` 或
`fast_livo2_lio`；操作员 session CLI 当前固定使用用户指定的默认
`fast_livo2_lio`。FAST-LIO2 profile 保留作历史只读参照，但选择策略拒绝启用它。
LIO-SAM loop 配置仅作实验 profile；温室重复行列数据上已观察到 false loop。

这一入口导出 FAST-LIVO2 frontend same-session reference，不运行 PGO。该 reference
保留成对的 patch 与 pose，可供后续离线建块/分析；它不能被当作独立 ground truth。
仓库、源码、历史基准和本次运行状态见
[`docs/backend/`](docs/backend/)，特别是
[`MULTI_LIO_BACKEND_ACCEPTANCE.md`](docs/backend/MULTI_LIO_BACKEND_ACCEPTANCE.md)。

## 技术边界

```text
MID-360 CustomMsg + IMU
          ↓
FAST-LIVO2 LIO-only (img_en=0, loop/PGO disabled)
          ↓
paired frontend body clouds + odometry
          ↓
verified source map package (map + patches + poses + provenance)
```

详细接口、地图格式、架构和研究扩展点见 [`docs/`](docs/)，交付验收记录见 [docs/delivery_acceptance.md](docs/delivery_acceptance.md)。本仓库独立完成建图、PCD→PGM、二维编辑和人工确认；确认后的地图可由其他运行时按需使用。

## 2026-09-24：新版栅格导出与地图发布

主仓库已整合 traversability 实验版，建图审阅默认使用 Bunker v1 配置。
新增 `mapping_map_release prepare/publish`，候选先放 experiments，人工验收后发布到 maps，
导航通过 registry 的 latest_validated 读取。
完整差异、测试、已生成候选和命令见 [整合报告](docs/mcp-traversability-main-integration.md)。
正式地图尚未切换；请先 source 工作区 install，再 source install_mapping_framework。
