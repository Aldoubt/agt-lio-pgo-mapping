# 建图实验版整合与地图交接报告

日期：2026-09-24。状态：**主仓库源码整合、正式建图 overlay 安装和离线验证完成；候选地图未发布到生产 maps，未进行实车验收。**

## 1. 实际路径与环境

本次通过 MCP 24.04 容器检查共享目录，并通过已有可信 SSH 连接到 Ubuntu 22.04 / ROS 2 Humble 主机执行构建和测试。两边本次可访问的实际路径均为 `/home/yangxuan/ros2_ws`；本次检查中 `/ros2_ws` 不存在。因此没有擅自新建 `/ros2_ws` 或向另一份目录写代码。

下文 `WS=~/ros2_ws`，仓库内部路径相对于 `src/agt_mapping_framework/`。

| 用途 | 路径 |
| --- | --- |
| 主仓库 | `$WS/src/agt_mapping_framework` |
| 实验版源码 | `$WS/experiments/mcp_traversability_20260924/worktree/agt_mapping_framework` |
| 导航仓库 | `$WS/src/agt_navigation_v3` |
| 正式地图交接目录 | `$WS/maps` |
| 正式建图安装 overlay | `$WS/install_mapping_framework` |
| 本次备份和测试记录 | `$WS/experiments/mcp_mapping_integration_20260924` |

## 2. 两版差异

主仓库 HEAD 为 `aac91bf`，实验 worktree 为 `d613a12`。实验版不是另一套 FAST-LIO2/PGO 建图核心，而是在同一建图结果上改进二维导航栅格导出。

| 环节 | 原主仓库 | 实验版及本次整合 |
| --- | --- | --- |
| FAST-LIO2 / PGO | 原有建图与优化流程 | 不改变算法、标定或参数 |
| 2D 导出 | local_ground、跨关键帧持久性过滤 | 新增 traversability 模式 |
| 自车点 | 缺少该模式的车体过滤 | 按 Bunker v1 足迹及高度过滤 |
| 地面 | 局部分位数地面 | 轨迹锚定地面生长及步高限制 |
| 空闲区域 | 主要依赖地面回波 | 增加射线证据、足迹扫掠和小孔填充 |
| 障碍证据 | 原持久性过滤 | 邻域支持、近场观测约束等 |
| 异常关键帧 | 原流程 | 倾角超过配置阈值时拒绝导出 |
| 调试输出 | 原统计 | 增加地面、射线和 traversability 统计 |

实验版 13 个文件的提交差异已选择性合入，没有整体覆盖主仓库。`exporters/agt_pcd2grid_exporter/` 与实验版本保持相同实现。

实验历史测评见 `docs/mcp-traversability-grid-upgrade.md`。其中规划成功率等数字是先前实验的记录，**不是本次重新运行规划器得到的结果**。

## 3. 本次额外衔接

### 默认使用新版进行建图结果审阅

`bringup/agt_mapping_bringup/agt_mapping_bringup/review.py` 默认显式选择 `projection_traversability.yaml`。

- `scripts/review_mapping_output.sh` 和原建图后审阅流程由此使用新版。
- 可用 `--config` 显式选择旧 `projection.yaml`。
- 独立 `pcd2grid_exporter` 未传配置时的原默认行为不变，避免破坏只输入 PCD 的调用者。
- Bunker v1 足迹和外参是本配置的适用条件，其他机器人不能直接照搬。

### 新增地图交接入口

新增 `mapping_map_release`，实现位于 `bringup/agt_mapping_bringup/agt_mapping_bringup/map_release.py`：

```text
已校验的优化 PGO 包
  → prepare：新版栅格导出
  → 同一 PCD 的重定位资产和候选描述子
  → experiments/mapping/candidates/<map-id>/<version>，状态 BUILT
  → 人工地图审阅、静态重定位及现场验收
  → publish --confirm-reviewed
  → maps/<map-id>/<version> + registry.yaml/latest_validated
  → 导航下一次启动按 auto 读取
```

`prepare` 校验源包完整校验和，不覆盖已有版本，不接触正式 maps；保存生成计划，并将导出参数、统计和源校验清单摘要放入 `converter_metadata.yaml`，随候选导航图交付。

`publish` 要求显式人工确认，复用已有 `agt_map_manager` 的锁、校验、版本不可覆盖和原子注册事务。不会默认传入 `--activate`，不会热切换正在运行的导航。

这里复用的是**已安装的地图管理和重定位工具 CLI**，没有复制一套注册表实现，也没有让导航重新生成二维地图。源码中的这些共享工具目前仍位于导航仓库；发布需要安装 `agt_map_manager` 和 `agt_global_relocalization_native`。本次没有把这两个包物理搬迁到建图仓库。

原 `scripts/export_validated_map_package.sh` 保持不变：它处理 mapping_source 包，不能视为此次完整导航候选包发布命令。

## 4. 导航读取规则

已核实导航 `scripts/run_field_stack.sh` 默认 `MAP_SPEC=auto`，通过 `resolve_map` 使用 `maps/registry.yaml`，并校验包哈希、VALIDATED 状态及机器人兼容性。

本次现场注册表已有：

```yaml
default_map: latest_validated
default_map_id: bunker_mid360
```

因此“最新”是**该地图 ID 最近一次成功验收发布所更新的指针**，不是目录 mtime，也不是对版本字符串排序。发布不同 map ID 不会自动改变 `default_map_id`。

两套入口当前不同：

- 新版 `auto`：`20260922_143427-v4-001`。
- 旧 `active_map.yaml`：`20260922_143427-fixed-replay-v1`。

本次不把它们强制同步。仍使用旧 active 入口的启动方式不会自动切到 latest_validated；需要使用已核实的新 field stack 入口。已运行的导航不会因发布自动换图。

## 5. 本次验证结果

| 验证 | 结果 |
| --- | --- |
| 主仓库源码在 Humble 构建 | exporter、Map Studio、bringup 均成功 |
| 正式建图 overlay 更新 | 同时重编译 exporter 和依赖其结构体的 Map Studio，避免 ABI 混用 |
| 三包测试 | exporter 18、Map Studio 23、bringup 90，均通过，无跳过 |
| colcon 汇总 | 隔离构建显示 139 项（含 8 个 CTest 包装结果），0 errors / failures / skipped |
| 正式构建复测 | 三包再次通过；目录总汇总 147，另含未在本次重跑的 refinement 历史 8 项，不能全计为本次新增验证 |
| 导航地图合同测试 | map_catalog、map_promotion_transaction 共 15 项通过 |
| 真实包全流程导出 | 702 关键帧，2719×1985 栅格，生成重定位资产及 702 个候选描述子 |
| 与实验版 v5 比较 | 新导出的 map.pgm 逐字节相同 |
| 旧模式回归 | 显式旧配置导出的 PGM 与旧 active 图逐字节相同 |
| 隔离发布和 auto 读取 | 在实验测试根目录发布成功，auto 正确解析 |
| 负例 | 错误机器人类型、篡改 PGM 均被拒绝；恢复后可重新解析 |
| 生产注册表保护 | registry.yaml、active_map.yaml、map_registry.yaml SHA-256 全部不变 |
| 原有未提交修改 | 5 个文件与修改前备份逐字节相同 |
| Git 检查 | diff --check 通过；未 commit、未 push |

新版 PGM SHA-256：

```text
d4b97fc2c0a8d2d4afbdc83341246b1e3395c28686c6c9ef2d05d6bd7b52ff93
```

本次使用现有优化 PGO 包验证导出，没有重新回放原始 bag 运行全程 FAST-LIO2/PGO，没有启动传感器、底盘、导航控制或机器人运动，也未重新测量几何精度、规划成功率和实际定位效果。

构建中的 OpenGL 选择和 Qt 弃用警告未阻止构建；其他建图包复用原有 underlay，不是全量冷构建。

## 6. 使用方式（22.04 主机）

先加载正确安装层，避免只加载旧 `install/`：

```bash
WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source "$WS/install_mapping_framework/setup.bash"
```

对新的优化建图包生成候选（版本必须唯一）：

```bash
ros2 run agt_mapping_bringup mapping_map_release prepare \
  --source /完整路径/到/map_package \
  --map-id bunker_mid360 \
  --version 20260924-trav-v2
```

可加 `--dry-run` 只校验源包和查看计划，不生成文件。失败的导出目录会保留用于排查；更换新版本重试，不自动删除。

本次已经生成、尚未发布的真实候选：

```text
~/ros2_ws/experiments/mapping/candidates/bunker_mid360/20260924-trav-integration-v1
```

其导航图片在该目录的 `navigation/map.pgm`，可检查对应 `map.yaml` 和 `converter_metadata.yaml`。原始导出和 debug 层在 `experiments/mapping/exports/bunker_mid360/20260924-trav-integration-v1/navigation/`。

**只有人工验收通过后**，再执行以下命令。它会更新新版 auto 的 latest_validated，影响下次导航选图；本次没有执行生产发布：

```bash
ros2 run agt_mapping_bringup mapping_map_release publish \
  --candidate "$WS/experiments/mapping/candidates/bunker_mid360/20260924-trav-integration-v1" \
  --confirm-reviewed
```

`--confirm-reviewed` 是操作人的验收声明，不是自动安全认证。先加 `--dry-run` 可查看命令。新发布仍需共享发布器完成结构与哈希校验。

只查看新版导航将选择哪张地图，不启动机器人：

```bash
ros2 run agt_map_manager resolve_map \
  --map auto --robot bunker_v1 --registry "$WS/maps/registry.yaml"
```

## 7. 安全边界与待验收项

实验版有意用历史轨迹足迹清空部分栅格；本次统计为清空 104 个已占据格。这有利于示教路径规划，但不能证明环境没有新障碍。近场过滤、树冠过滤和路沿地面判断同样有适用边界。应重点审阅窄通道、路沿、低悬物体和负障碍区域，并完成静态重定位与受控低速验收。

未观测区域不能仅因地图看起来更清楚就认定安全可通行。本次不替代现场验收，不修改导航避障参数。

## 8. 备份与回滚

本次工作目录 `experiments/mcp_mapping_integration_20260924/` 中保存：

- `before-source.tar.gz`：主仓库修改前源码，包含原有未提交改动，不包含 `.git`。
- `preexisting.patch`、`preexisting-status.txt`、`before-head.txt`：修改前 Git 状态。
- `traversability.patch`：选择性合入的实验补丁。
- `before-install-mapping-framework-host.tar.gz`：在主机端完成的有效安装层备份。
- `maps-before.sha256`：正式地图指针校验值。
- 构建、测试、实际导出、隔离发布结果及 dry-run 计划。

注意：`before-install-mapping-framework.tar.gz` 是容器端首次尝试的**不完整备份，不用于恢复**；容器无法解引用部分指向主机 ROS 的符号链接。有效备份是带 `-host` 的文件。

源码回滚应只恢复本次变更的文件，并移除本次新增文件；不要 `git reset --hard` 或全仓覆盖，以免丢失原先的 5 个未提交修改及本次之后的工作。可先将源码备份解压到独立目录进行比较。

安装层回滚应在没有使用该建图 overlay 的进程时进行：先把当前 `install_mapping_framework` 移到备份目录，再在主机端恢复 `before-install-mapping-framework-host.tar.gz`，开启新终端重新 source。不要覆盖仍在运行的程序。

正式地图未修改，因此本次不需要地图回滚。未来若发布新版本后要回退，新 field stack 可显式指定此前的 `map-id/version`；单改旧 active 指针不会改变默认 latest_validated 选图。
