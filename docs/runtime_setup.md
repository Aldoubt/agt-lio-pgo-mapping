# MID360 PGO 建图操作指南（编排 0.2.0）

## 环境与构建

目标环境：Ubuntu 22.04 / ROS 2 Humble。**ROS 命令必须在宿主机终端或已授权的宿主执行会话中运行；共享代码目录不等于共享容器与宿主的 ROS 运行环境。**`--help` 只需要 Python 3；`--dry-run` 和独立地图校验还需要 PyYAML（`python3-yaml`），不需要运行 ROS 节点。

新工作区仍使用 `scripts/bootstrap.sh` 获取锁定外部依赖并构建。不要把同一份 `.repos` 重复导入工作区其他目录，以免出现重复包名。

已有 mapping overlay 的增量构建示例（在工作区根目录，在真实 Humble 环境执行）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash  # 本机完整 C++ 建图依赖的 underlay
colcon --log-base log_mapping_framework build \
  --base-paths src/agt_mapping_framework \
  --build-base build_mapping_framework \
  --install-base install_mapping_framework \
  --packages-select agt_mapping_artifacts agt_mapping_exporter agt_mapping_bringup \
  --allow-overriding agt_mapping_artifacts agt_mapping_exporter agt_mapping_bringup
```

若平时使用 `install/`，应替换为对应的构建/安装目录，并在运行时显式指定 `--setup`。入口优先选择 `install_mapping_framework/setup.bash`，其次 `install/setup.bash`；缺少新版辅助 executable 时会拒绝启动并提示重建，不会默默运行旧流程。

> 2026-09-20 已确认：MCP 在 Ubuntu 24.04 Docker 中运行，目录共享给 Ubuntu 22.04.5 / Humble 宿主。现已通过宿主测试，详见 `docs/mcp-host-validation.md`。本机单独 source `install_mapping_framework/setup.bash` 不能找到全部后端，构建时应先 source 完整 `install/setup.bash`。不要因此重装 ROS 或重复导入外部依赖。

本轮已构建的独立测试安装可直接使用，无需覆盖现有安装：

```bash
WS=/home/yangxuan/ros2_ws
TEST_SETUP="$WS/experiments/mcp_host_mapping_validation/20260920_082032_d2e0a6/install/setup.bash"
cd "$WS/src/agt_mapping_framework"
./scripts/run_mid360_mapping.sh /path/to/mid360_bag --setup "$TEST_SETUP" --no-rviz
```

这个基准包包含原始与过滤后两路 CustomMsg，运行它时明确追加 `--lidar-topic /agt/sensors/lidar/custom --imu-topic /agt/sensors/imu/data`，不要误用过滤支路。测试输出目录始终使用新的/空的目录。

## 最少操作路径

从仓库根目录执行：

```bash
# 只读检查输入、话题和输出目录；打印 JSON 执行计划，不启动节点/创建输出目录。
./scripts/run_mid360_mapping.sh /path/to/mid360_bag --dry-run

# 一条命令：预检 -> 就绪门控 -> 回放 -> PGO 导出 -> 完整性校验 -> 自动关闭本次 launch。
./scripts/run_mid360_mapping.sh /path/to/mid360_bag

# 兼容原来的两个位置参数；输出目录必须是新目录或空目录。
./scripts/run_mid360_mapping.sh /path/to/mid360_bag /data/agt_mapping_runs/run_001
```

默认根据 `DISPLAY`/`WAYLAND_DISPLAY` 判断是否打开 RViz，也可以明确使用 `--rviz` / `--no-rviz`。RViz 显示的是**实时 LIO 点云/轨迹，不是最终 PGO 地图**。最终交付以 `map_package` 和校验结果为准。

## 常用选项

| 选项 | 用途 |
| --- | --- |
| `--rate 0.5` / `--rate 2` | 回放速度；过快可能超过计算能力，验收建议先用 1.0 |
| `--no-rviz` / `--headless` | SSH/无图形环境运行 |
| `--start-paused` | 图就绪后创建暂停状态的 player，待操作者恢复 |
| `--keep-open` | 校验完成后保留节点/RViz，由操作者 Ctrl+C 关闭 |
| `--manual-export` | 回放结束后保留节点，不自动请求导出、不宣告自动完成 |
| `--lidar-topic TOPIC` | 显式选择 `livox_ros_driver2/msg/CustomMsg` 输入 |
| `--imu-topic TOPIC` | 显式选择 `sensor_msgs/msg/Imu` 输入 |
| `--startup-timeout 60` | 就绪门控墙钟超时，默认 45 秒 |
| `--drain-seconds 3` | 回放结束后的前端静默窗口；不是算法内部队列排空证明 |
| `--export-timeout 300` | 排空、导出应答及产物等待的总墙钟期限，默认 180 秒 |
| `--setup /path/to/setup.bash` | 明确选择已重建的工作区 overlay |
| `--ros-setup /path/to/setup.bash` | 基础 ROS 环境路径 |
| `--domain-id 90` | 独立本地 ROS domain，默认 89；同时运行另一任务需使用不同 domain |

自动话题识别只在每种传感器类型各有一个**非空流**时选择。存在多个同类输入时要求显式指定，缺少 IMU/CustomMsg 时直接拒绝启动，不能靠指定一个不存在的话题补救。

FAST-LIO2 的输入来自 YAML，而不是同名 ROS 参数。新版读取已安装的 FAST-LIO2 配置并使用 ROS remapping 适配记录的输入话题；不会改写配置文件、外参或外部依赖代码。原始点云仍先经过现有适配器，再进入 `/mapping/sensor/livox`，不插入体素/自过滤。

## 暂停、恢复与取消

入口默认设置 `ROS_DOMAIN_ID=89`、`ROS_LOCALHOST_ONLY=1`，避免离线回放混入默认机器人 ROS 网络。控制命令必须 source **同一环境**并使用同一 domain。每个 domain 只运行一套建图任务；本用户使用新版入口的同 domain 并发会被进程锁拒绝。

```bash
# 另一个已 source Humble + 同一 overlay 的终端：
ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 ros2 service call \
  /rosbag2_player/pause rosbag2_interfaces/srv/Pause '{}'
ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 ros2 service call \
  /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'
```

- 主终端 Ctrl+C：取消本次 launch，不触发新的自动导出，不宣告完成。
- 如果在导出已开始后取消，磁盘可能留下未完成文件；保留供排查，不自动删除，也不能当作可交付地图。
- 不使用 `pkill` 或终止其他 ROS 进程；由 launch 关闭自己的子进程。
- domain 锁随 launch 进程退出释放，包括子进程清理阶段；空锁文件保留属于正常现象，不需要手动删除。
- 使用旧版入口或其他程序时，也必须保证该 domain 没有同名建图服务/节点。进程锁不是跨主机分布式锁。

## 完成判据与产物

运行输出：

```text
run_001/
├── session.json          # 输入、选项、阶段、失败原因和最终校验状态
├── pgo_raw/              # 后端原始导出
└── map_package/
    ├── map.pcd
    ├── poses.txt
    ├── poses_timed.txt
    ├── patches/
    ├── calibration.yaml
    ├── metadata.yaml
    ├── manifest.yaml
    └── checksums.sha256
```

自动流程只有在终端出现 `[4/4] Artifact VERIFIED`、`session.json` 为 `status: completed` 且 `artifact_verified: true` 时才完成。Humble launch 的正常 Ctrl+C 取消可能也返回退出码 0，因此不能只看退出码。Trigger 的 `success: true` 只是后端受理请求，不能代替最终完成。

独立只读复核：

```bash
./scripts/verify_map_artifact.sh /data/agt_mapping_runs/run_001
# 或直接指定 .../map_package
```

校验包括必需文件非空、PGO/optimized 布尔契约、非空 PCD 头/数据、patch 存在、完整文件清单以及 SHA-256。未列入 checksum 的文件、缺失文件、越界路径、符号链接、空地图或校验不一致均拒绝交付。这不等于地图几何精度/闭环质量验收。

当前后端状态协议手工拼接 JSON，因此输出路径暂不支持双引号、反斜杠或换行；空格路径受支持。输出目录不允许复用非空目录，不覆盖旧地图，不向输入 bag 写入。

## 高级：手动导出与直接 launch

```bash
./scripts/run_mid360_mapping.sh /path/to/bag --manual-export --keep-open
# 在同一 ROS domain 的另一个终端，回放完成后：
ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 ros2 service call \
  /mapping/backend/export_artifact std_srvs/srv/Trigger '{}'
# 然后独立校验产物；手动模式不会自动将 session 状态标成 completed。
```

直接 `ros2 launch agt_mapping_bringup mapping_v0.launch.py` 保留原入口，但需自行设置隔离环境、传入 `bag_path:=... output_dir:=...`，且已 source 新版 overlay。新增 launch 参数与 CLI 对应：`playback_rate`、`start_paused`、`keep_open`、`startup_timeout`、`export_timeout`、`drain_seconds`。`auto_export:=false` 保留手动模式。

## 常见失败与测试

- **缺 ROS/setup 或 stale overlay**：先使用正确 Humble 环境，按上文增量构建；不要把已有 install 目录当成已安装运行时。
- **缺存储分片/IMU/CustomMsg**：修复输入记录，不能继续空跑建图。
- **就绪超时**：查看消息中的缺失服务/订阅者；bag 尚未开始回放。
- **回放失败**：不会继续自动导出；查看 player 错误及 `session.json`。
- **后端拒绝或产物超时**：检查 PGO/前端状态、计算负载和实际输出；不取消前端时效门限、不自动重发可能已受理的导出。
- **静默窗口超时**：前端仍持续处理或图中有其他数据源；先检查负载/domain，而不是无条件忽略。

独立回归（Python 3 + PyYAML，不依赖 ROS）：

```bash
./scripts/test_mapping_workflow.sh
```

其中 fake graph/service/launch contract 测试不是真实 ROS 集成测试；新增的真实 launch SDK 用例在无 ROS 时会明确跳过。在宿主机使用 `colcon test` 可覆盖三个改动包及 exporter 生命周期。本次宿主 colcon 82 项无跳过通过，完整录包、原始话题派生回放、取消/失败及暂停恢复已实测；RViz 界面、实车和地图几何质量仍需单独验收，详见 `docs/mcp-host-validation.md`。
