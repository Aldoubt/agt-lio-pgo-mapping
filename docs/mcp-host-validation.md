# 共享目录宿主机测试报告

日期：2026-09-20。仓库：`agt_mapping_framework`。编排版本：0.2.0。

## 结论

**可以测试，而且已在真正的 Ubuntu 22.04 / ROS 2 Humble 宿主机完成构建、82 项自动化测试、完整基准录包回放和操作场景验证。**

前一阶段看到的 Ubuntu 24.04 是 MCP 命令所在的 Docker 容器，不能据此判断宿主机缺少 ROS。用户说明共享关系后，已通过用户在本地终端完成认证的 SSH 会话核实宿主机并执行测试。

- 测试安装在独立目录，未覆盖原有 `install/`、`install_mapping_framework/`。
- 原始录包以 READ_ONLY 打开；原 metadata SHA-256 未变；新地图和派生录包均写入新实验目录。
- 未启动真实传感器驱动、CAN、底盘或运动功能，未发布机器人速度指令。
- 测试 domain 97、98 已确认没有遗留节点；临时 SSH 会话已软关闭，未使用强制停止。
- 代码保留在工作树，未提交、未推送。未修改外部 FAST-LIO2/PGO 算法或标定参数。

## 环境边界与执行入口

| 项目 | 已核实结果 |
| --- | --- |
| MCP 执行环境 | Docker，Ubuntu 24.04.5，Python 3.12；不含 ROS 运行时 |
| 共享目录 | `/home/yangxuan/ros2_ws`，容器内是可写的宿主 ext4 挂载 |
| 宿主机 | Ubuntu 22.04.5，Python 3.10.12，ROS 2 Humble |
| 宿主工具 | colcon、CMake、g++、rclpy、launch、rosbag2_py、PyYAML、pytest 可用 |
| 宿主入口 | 本机 SSH，经用户核对指纹并在 ShunCode 终端输入密码认证 |
| 完整运行 underlay | 工作区 `install/setup.bash` |
| 环境注意事项 | 单独 source 当前 `install_mapping_framework/setup.bash` 时并不能找到全部建图后端；不要仅凭目录存在认定环境完整 |

没有在聊天中索取密码，没有配置免密、转发 SSH agent、开启端口转发或修改 sshd。认证只用于本次测试，结束后已关闭连接。

## 测试安装与产物位置

以下路径相对于工作区根目录（不是仓库根目录）：

```text
experiments/mcp_host_mapping_validation/20260920_082032_d2e0a6/
├── build/、install/             # 独立测试构建和安装
├── build-final.log
├── colcon-test-final.log
├── test-result-final.log
├── tested-source-sha256.json
├── baseline-bag-info.txt
├── baseline-run.log
├── baseline-metrics.json
├── baseline_205036/map_package/ # 完整基准地图
├── pause-resume-result.json
├── edge_cases.py
├── edge_cases_final/            # 坏录包与取消场景记录
├── raw-topics-provenance.json
├── raw_topics_60s_v2/           # 60 秒窗口、仅改话题名的派生数据
├── raw-topic-run.log
├── raw-topic-result.json
├── raw_topic_run_final/map_package/
└── cleanup-domain-97.json、cleanup-domain-98.json
```

初次派生探针曾因 metadata 时间字段名假设错误而中止；`raw_topics_60s/` 是该次空的失败尝试，不作为有效测试输入。有效派生输入是 `raw_topics_60s_v2/`，时间窗由记录消息自身时间戳确定。

## 构建与自动化测试

只增量构建三个变动 Python 包，C++ 后端复用宿主机现有安装，未进行全量冷构建：

| 包 | colcon 测试结果 |
| --- | --- |
| `agt_mapping_artifacts` | 16 passed |
| `agt_mapping_exporter` | 5 passed |
| `agt_mapping_bringup` | 61 passed |
| **合计** | **82 tests，0 errors，0 failures，0 skipped** |

真实 Humble SDK 下的 launch 文件导入、动作/事件组合和异常动作均通过，`mapping_v0.launch.py --show-args` 正常。

最终安装层中的 14 个 Python 模块与当前源文件逐字节一致；另记录了两份建图 YAML 配置的 SHA-256。构建日志中 byte-compiling disabled 的 warning 来自测试环境的 `PYTHONDONTWRITEBYTECODE=1`，不是编译失败。

## 真实录包回归

### 1. 完整主基准

输入为仓库 `docs/baseline_v0.1.md` 指定的录包：

```text
experiments/data/rosbag/bunker_mid360_mapping_20260901_205036
```

- metadata SHA-256：`7856c31a842d417afd503d887375adc5caa62263acd656d1fbb88c4a27439206`。
- 录包总时长 482.297427344 秒，记录 268,427 条消息，SQLite 文件 3,318,849,536 字节。
- 明确选择原始 `/agt/sensors/lidar/custom` 与 `/agt/sensors/imu/data`。
- 该包还有 `custom_filtered`，因此不使用模糊的自动选择，不将过滤支路误送入 LIO。
- 1.0 倍速、无 RViz、ROS domain 97、`ROS_LOCALHOST_ONLY=1`。
- 整条验证命令约 491.5 秒，退出码 0；最终 session 为 `completed`、`artifact_verified: true`。

结果：

| 指标 | 本次结果 |
| --- | --- |
| 优化关键帧/轨迹记录 | 333 |
| 地图点数 | 674,244 |
| 轨迹覆盖 | 444.599 秒 |
| 最大相邻轨迹步长 | 0.568052 米 |
| 最大推算速度 | 0.598657 米/秒 |
| map.pcd 大小 | 10,788,094 字节 |
| 完整文件覆盖与 SHA-256 | 通过 |
| 自动结束与节点回收 | 通过 |

本次点数并非与历史交付记录 673,897 点逐点相同，未据此宣称地图几何质量完全等价。日志保留了已有适配器丢弃 1 条 non-aggregate Livox 消息的警告，未为通过测试放宽门限或更改算法参数。

### 2. 暂停/恢复

在上述真实回放中，通过真实 ROS 服务验证：

- 3 秒内收到 31 条前端里程计和 30 条前端点云消息。
- IsPaused：`false → pause → true → resume → false`。
- 测试图中没有 `/cmd_vel`、`/mux/cmd_vel` 或 `/bunker/cmd_vel`。
- 结果保存在 `pause-resume-result.json`。

### 3. 坏录包与取消

在另一空闲本机 domain 98 中运行真实 launch，未使用模拟的 launch 对象：

| 场景 | 结果 |
| --- | --- |
| 有合法 metadata、但 SQLite 内容损坏的自建负例 | 退出码 1，状态 `failed`，未进入导出阶段，无 `map_package`/`pgo_raw`，节点清理完成 |
| 原始录包 `--start-paused`，确认真实 IsPaused 为 true 后中断本测试创建的 launch | 状态 `cancelled`，没有导出，节点清理完成 |
| 清理方式 | 未发生强制清理；不使用全局 pkill 或终止其他 ROS 进程 |

注意：Humble launch 在正常 Ctrl+C 取消时可能返回 0。本次取消用例即如此，**不能仅凭退出码 0 判断地图交付成功**；必须同时检查 session 状态和产物验证。

### 4. 最终代码的原始话题兼容与干净退出

完整基准发现正常退出时旧导出节点会打印 KeyboardInterrupt，随后已修复其生命周期收尾。为验证最后一次修改，同时验证实际话题适配，又运行了最终代码：

- 从原始录包前 60 秒窗口复制传感器序列化数据到新目录，原始测量字节及时间戳不变。
- 只把话题名改为 `/livox/lidar` 和 `/livox/imu`。
- 派生输入有 579 条 CustomMsg、11,574 条 IMU；没有复制控制话题。
- 不显式指定话题，让入口自动检测，验证实际 FAST-LIO2 的 YAML 输入被 ROS remapping 正确适配。
- 生成 26 个优化关键帧，最终产物验证通过，退出码 0。
- `raw-topic-run.log` 无 `[ERROR]`、Traceback 或 KeyboardInterrupt；导出节点和其余节点均正常退出。

**版本覆盖说明：完整 482 秒回放发生在 Humble API 修复之后、导出节点正常退出修复之前。最终代码随后通过 82 项测试、真实失败/取消场景和上述派生数据回放；没有把较早完整回放冒充为最终代码再次执行完整 482 秒。**

## 实测发现并落地的修复

1. `session_launch.py`：Humble 不提供 `launch.actions.RaiseError`。改用 Humble 支持的 `OpaqueFunction` 抛出异常，保留错误路径的非零退出，并增加 `test_ros_launch_sdk.py` 的真实 SDK 检查。
2. artifact 包此前 `colcon test` 会收集 0 项测试。补齐 `tests_require=['pytest']`，使 16 项测试真正执行；exporter 包也补齐测试发现配置。
3. `exporter_node.py`：正常 SIGINT/ROS context shutdown 不再打印异常栈；在 finally 中清理节点及仍存活的 context。非预期异常仍继续抛出，新增 4 个生命周期测试。

## 复现方式

以下命令必须在宿主机 22.04 终端执行，或使用已授权的宿主机执行会话，不能直接在缺 ROS 的 MCP 容器内运行：

```bash
WS=/home/yangxuan/ros2_ws
TEST="$WS/experiments/mcp_host_mapping_validation/20260920_082032_d2e0a6"
cd "$WS/src/agt_mapping_framework"

# OUTPUT_NEW 必须换成新的/空的目录，不能覆盖本次验收产物。
./scripts/run_mid360_mapping.sh \
  "$WS/experiments/data/rosbag/bunker_mid360_mapping_20260901_205036" \
  /path/to/OUTPUT_NEW \
  --setup "$TEST/install/setup.bash" \
  --lidar-topic /agt/sensors/lidar/custom --imu-topic /agt/sensors/imu/data \
  --no-rviz --rate 1.0 --domain-id 97
```

构建、更多选项和取消说明见 `docs/runtime_setup.md`。本次测试安装层没有替代正式安装层；当前机器请显式传入上述 `--setup`，或者按操作指南在完整 underlay 上重新构建正式 mapping overlay。

## 验收范围之外

本轮通过的是 **宿主机离线建图工作流验证**。未做 RViz 实际界面验收、实机传感器/车辆测试、导航地图发布或几何精度验收。手动导出、keep-open 和真实后端超时故障没有逐项做现场集成验收；已有逻辑测试不代替这些实测。
