# 建图操作流程升级：审查、方案与验证

日期：2026-09-20。范围：`agt_mapping_framework` 的离线 MID360 rosbag 建图入口。

## 交付结论

**当前结论已更新：编排 0.2.0 已在 Ubuntu 22.04 / Humble 宿主机完成构建、82 项自动化测试和离线录包回归。此前环境阻塞只存在于 MCP 的 24.04 容器。最新证据与版本覆盖见 `docs/mcp-host-validation.md`；这仍不等于实车或地图几何精度验收。**

入口保持 `scripts/run_mid360_mapping.sh BAG [OUTPUT]`，新增预检、原始/归一化话题适配、就绪门控、受控导出、完整性校验、运行记录与自动收尾。操作指南：`docs/runtime_setup.md`。

首次容器阶段交付为 22 个新增/修改文件（7 个已跟踪文件修改、15 个新文件）；本轮又补充了宿主实测发现的修复与测试，见新报告。未提交、未推送，未改外部算法/驱动仓库、原始 bag、已有地图包或机器人控制逻辑。

## 仓库识别与授权范围

- 工作区目录：`src/agt_mapping_framework`，仓库标题为 AGT LIO–PGO Mapping。
- 开始时分支为 `main`，HEAD 为 `7d4ff8e`，工作树干净。
- 用户选择由代理审查现有工作流、确定升级范围并实施。
- 建图链路：MID360 CustomMsg/IMU → 传感器适配 → FAST-LIO2 → PGO → 地图包。
- `apps/agt_map_studio` 是离线地图查看/编辑工具，不是实时建图控制器。
- 本次只升级编排层及交付校验，不改算法参数、传感器驱动、外部锁定依赖、导航或底盘控制，不启动实机。

## 已确认的问题

| 原位置 | 证据与影响 | 本次处理 |
| --- | --- | --- |
| `scripts/run_mid360_mapping.sh` | 参数仅有 bag/输出目录；在参数校验前 source 固定 Humble 环境；帮助也依赖 ROS | ROS 无关帮助/预检，保留两个位置参数，支持调速、无 RViz、暂停启动、手动导出及超时配置 |
| 同上 | 只检查 bag 目录存在，不检查 metadata、实际存储分片或传感器话题 | 读取 rosbag2 metadata，检查所有分片，按消息类型识别唯一输入；歧义要求显式选择 |
| `bringup/agt_mapping_bringup/launch/mapping_v0.launch.py` | 固定等待 5 秒后播放；图和服务未就绪也会开始 | 有界、基于真实订阅/服务发现的启动门控 |
| 同上 | `OnProcessExit` 没有区分退出码或 shutdown，回放失败也会尝试导出 | 正常结束才导出；失败、取消时停止并保留原因 |
| 同上与外部 FAST-LIO2 源码 | FAST-LIO2 只从 YAML 读取话题；原 launch 的 imu_topic ROS 参数无效，原始 `/livox/imu` bag 与配置不匹配 | 从既有 YAML 提取输入话题并显式 ROS 重映射，不改外部代码或标定 |
| 同上及 `backends/agt_pgo_backend/src/pgo_backend_node.cpp` | Trigger success 只表示异步导出已受理，不代表产物已完成 | 等待最终产物，经过内容、完整清单和 SHA-256 校验后才显示完成 |
| `scripts/verify_map_artifact.sh` | 只检查现有 checksum 条目；未要求清单完整，也不拒绝空点云 | 共享只读校验器，拒绝空地图、缺文件、遗漏 checksum、越界路径和符号链接 |
| 编排层 | 完成后没有统一退出/状态记录；用户容易过早关 RViz 或不知道是否已保存 | 阶段提示、`session.json`、默认成功后自动退出，可显式保留窗口 |

## 设计与兼容性

- 编排包升级为 0.2.0；地图包格式不变。
- `scripts/run_mid360_mapping.sh BAG [OUTPUT]` 保持兼容。
- `preflight.py` 管理只读输入验证；`session_state.py` 管理运行记录和退出策略。
- `session_runtime.py` 只做 ROS 就绪门控和有界导出等待；不承担算法功能。
- `session_launch.py` 负责进程生命周期；launch 文件只声明参数和组合。
- `artifacts/agt_mapping_artifacts/agt_mapping_artifacts/validation.py` 是 CLI 与自动导出共用的产物校验器。
- 输出目录必须是新目录或空目录，独占创建运行记录，不覆盖旧地图或向输入 bag 写文件。
- wrapper 默认使用本机回环及独立 ROS domain 89，支持 `--domain-id` 显式指定。本用户的新版建图进程通过 domain 文件锁防止并发混流；锁持有至 launch 进程退出，不杀停已有任务。手动调用 launch 时需自行设置隔离环境。
- 启动、导出等待使用单调墙钟，不依赖回放结束后停止前进的 `/clock`。
- 现有后端没有可证明全队列排空的 API；保留可调的排空静默窗口，不声称实现了算法内部队列屏障。
- `--manual-export` 保留人工高级工作流，不自动宣告成功或关闭节点。

## 首轮容器验证记录与后续更正

首轮检查的执行环境是 Ubuntu 24.04.5 / Python 3.12.3，未发现 Humble/colcon；后续证实那是 MCP Docker 容器，并非宿主机。共享目录的实际宿主为 Ubuntu 22.04.5 / Python 3.10.12 / Humble，运行工具齐全。以下保留首轮容器测试记录，新增宿主验证另见 `docs/mcp-host-validation.md`。

不会未经要求安装整套 ROS、升级系统或重建外部依赖。

| 验证 | 当前状态 |
| --- | --- |
| 输入/输出预检、domain 防并发及运行状态策略 | 29 项通过 |
| 产物正例、损坏、空地图、不完整清单、路径安全与校验超时 | 14 项通过 |
| CLI 帮助、dry-run、参数、argv 边界与错误提示 | 8 项通过 |
| launch 生命周期 contract（stand-in 对象） | 12 项通过；不是真实 ROS 集成 |
| 就绪/异步导出 contract（fake graph/service） | 9 项通过；不是真实 ROS 集成 |
| Python/Bash 语法与补丁空白检查 | 22 个 Python 文件通过 3.10 grammar 检查；4 个脚本 bash -n 通过；git diff --check 通过 |
| ROS launch 接口、就绪发现和退出事件真实运行 | 首轮容器受阻；现已通过宿主 SDK 与真实流程验证，见新报告 |
| Humble colcon 构建及真实 rosbag 回归 | 现已通过宿主测试；82 项无跳过通过，并完成完整基准回放 |
| 实机传感器/车辆验证 | 本次不执行 |

单元测试、结构测试不等于真实 bag 回归，更不等于车辆验收。本节以下 72 项结果是在 MCP 执行容器 Python 3.12.3 上取得，当时对 3.10 仅做语法检查。新报告中的 82 项测试则是在宿主 Python 3.10.12 / Humble 上真实执行。

### 已执行验证与证据

1. **独立回归**：`scripts/test_mapping_workflow.sh`，bringup 58 项 + artifact 14 项，共 72 项，通过，进程退出码 0。
   - 服务返回 `success: true`、但尚无文件时必须超时而非报成功。
   - 服务拒绝、异步 PGO 失败、损坏产物、持续前端积压均不进入 completed。
   - 回放非零退出、shutdown 即使碰上 player 返回 0，也不触发导出。
   - FAST-LIO2 使用显式 remapping，而不是无效的同名 ROS 参数。
   - 同 domain 重入被拒绝，空锁文件无需删除，不接触其他运行中的进程。
2. **无 ROS 帮助**：`bash scripts/run_mid360_mapping.sh --help` 成功；测试子进程清除了测试 PyYAML 的 PYTHONPATH，帮助仍能输出。
3. **正例预检**：用包含完整 metadata/分片和原始/归一化传感器话题的合成 fixture 验证 dry-run 成功、参数正确且不创建输出目录。这不是实际 rosbag 解码或 SLAM 回放。
4. **现有录包只读负例**（工作区相对路径，不修改原数据）：
   - `agt_data/field_acceptance/nav_test_001`：没有非空 `sensor_msgs/msg/Imu` 输入，拒绝，退出码 2。
   - `experiments/results/yaw_analysis_211105/replay_bag` 和 `replay_bag_no_wheel_input`：没有非空 `livox_ros_driver2/msg/CustomMsg` 输入，拒绝，退出码 2。
   - 仅说明这些抽查样本不满足本入口的输入契约，不代表工作区没有其他合适的 bag。
5. **现有真实 PGO 产物只读复核**：工作区 `experiments/artifacts/output/mid360_20260901_205036_validated_v2/map_package`，完整覆盖与 SHA-256 校验通过，退出码 0。未重新生成该地图，也不能据此声称新版建图链路已回归。
6. **结构检查**：22 个 Python 文件用 `ast.parse(..., feature_version=(3, 10))` 通过；package.xml/setup.py 版本一致为 0.2.0；4 个改动脚本通过 `bash -n`；补丁检查通过。
7. **编辑器 diagnostics**：快照返回 0 条，但服务明确标记 inconclusive，不用它作为项目无错误的证明。

### 复现独立验证

标准环境安装 PyYAML 后，从仓库根目录：

```bash
./scripts/test_mapping_workflow.sh
```

首轮 MCP 执行容器没有系统 PyYAML/pip（宿主机已有）。仅将经 SHA-256 校验的 PyPI PyYAML 6.0.2 wheel 中 `yaml/` 提取到工作区 `experiments/mcp_mapping_upgrade_validation/python_deps`，未安装系统软件、未改 ROS 环境。可在本机重现：

```bash
PYTHONPATH="../../experiments/mcp_mapping_upgrade_validation/python_deps${PYTHONPATH:+:$PYTHONPATH}"   ./scripts/test_mapping_workflow.sh
```

原始日志与依赖记录位于工作区 `experiments/mcp_mapping_upgrade_validation/`，不放入仓库 `docs/`：

- `unit-tests.log`：完整 72 项测试日志。
- `dependency.json`：PyYAML 版本、wheel 名称及 SHA-256。
- `existing-artifact-verification.log`：已有 PGO 地图包复核结果。
- `replay_bag-preflight.err`、`replay_bag_no_wheel_input-preflight.err`：输入拒绝原因。

### 发布检查表与剩余边界

以下是最初计划。第 1、2 项以及第 3 项中的正常收尾、回放失败、暂停取消现已完成宿主验证；真实后端超时、手动模式、GUI 和地图质量不能据此自动判为通过：

1. 按 `docs/runtime_setup.md` 重建 `agt_mapping_bringup`、`agt_mapping_artifacts`，确认新版 entry points 和依赖安装。
2. 对实际包含 CustomMsg + IMU 的原始录包进行完整、隔离 domain 的回放，确认节点图就绪、真实 remapping 和最终非空地图。
3. 验证正常完成自动收尾、播放失败、播放中取消、导出超时以及手动模式。
4. 有图形环境时验收 RViz、暂停/恢复和 keep-open 体验。
5. 对地图几何、闭环和定位资产另做既有质量验收；文件完整性不替代地图质量。

未执行实机传感器、CAN、底盘或运动操作，也未安装 ROS/colcon。首轮容器阶段未调用建图 ROS 节点；后续已在宿主隔离 domain 中执行离线建图并确认节点清理。任务卡中的宿主构建/离线验证项已完成；尚未覆盖的发布边界以新报告为准。
