# 导航栅格地图升级：可通行性证据模型（traversability 模式）

日期：2026-09-24。分支：`feature/mcp-traversability-grid`（基于 `aac91bf`，独立 worktree）。
范围：`exporters/agt_pcd2grid_exporter`。不改 FAST-LIO2/PGO、地图包格式、`maps/`、
`maps/active_map.yaml`、导航参数或机器人控制；未上车验证。

## 交付结论

- 新增 `projection_mode: traversability` 与 `config/projection_traversability.yaml`。
  默认 `projection.yaml`（`local_ground`）行为不变：用新代码按默认配置重新导出当前激活地图，
  `map.pgm` 与现网文件逐字节一致。
- 新模式从关键帧包（`patches/*.pcd` + `poses_timed.txt`）按“证据”建图：自车滤波、
  轨迹锚定的地面生长、传感器射线雕刻空闲空间、持久障碍 + 邻域支撑、轨迹足迹扫掠、
  近场观测过滤、发散位姿拒绝。
- 当前激活地图对应的包（0922 fixed replay）：Nav2 规划成功率 20% → 100%，
  轨迹落入内切碰撞区 44.0% → 0%，之字形 6.2 → 3.8 次/10 m，轨迹 5 m 内未知 28.3% → 17.3%。
- 另外 3 个有效建图包同样全部达到 100% 规划成功（原 45%～62.5%）；2 个发散的包被明确拒绝。
- 单元测试 18/18（原 8 + 新 10），依赖本库的 `agt_map_studio` 重编译后 22/22。

## 两个仓库的方法（现状）

建图（`agt_mapping_framework`）：MID-360 → `agt_mid360_adapter` → FAST-LIO2
（`lidar_filter_num 6`、0.5–30 m、`scan_resolution 0.15`、`gravity_align`）→ 关键帧（0.5 m/10°）
+ PGO → 地图包（机体系 patch + `poses_timed.txt` + `map.pcd`）→ `agt_pcd2grid_exporter`
（`local_ground` + 跨关键帧持久性过滤）→ `navigation/map.pgm` → `agt_map_manager create_map_package`。

导航（`agt_navigation_v3`）：Batch-LIO / FAST-LIO2 局部里程计；3D-BBS + small_gicp 全局重定位；
`agt_localization_manager` 发布 `map→odom`；Nav2（无 AMCL），SmacPlanner2D + RPP →
velocity smoother → `cmd_vel_guard` → Bunker。全局代价图仅 static + inflation
（1.0 m，scaling 2.0，`track_unknown`），`cost_travel_multiplier 3.0`，`allow_unknown true`，
足迹 ±0.52 × ±0.40 m、padding 0.03。自 `e27abeb`（09-22）起行为树为
`navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml`。
另有 `agt_map_converter/pcd_to_nav_map`（Map Studio 发布流程使用），当前激活地图不是由它生成。

## 根因（均有数据支撑）

| # | 问题 | 证据 | 对规划的影响 |
| --- | --- | --- | --- |
| R1 | 车体自身回波被烘进静态图 | 相机杆位于 base_footprint (-0.10～-0.21, 0～0.05, 1.04～1.75 m)，出现在 67.7% 的关键帧；导出器无自车滤波，跨帧持久性过滤挡不住（它随车移动但每帧都在） | 行驶轨迹上周期性“T”形占据：44.0% 轨迹采样距占据格 < 0.43 m（82 段）；起终点落入致死区 → 规划失败；路径绕“T”左右摆 |
| R2 | 空闲只来自地面回波，无射线推理，输入又稀疏 | 约 1/72 原始点进入地图（每 patch ~3000 点）；轨迹 10 m 内未知 51.4%，椒盐指数 14.7%，10 m 内 4218 个未知碎块 | 未知格代价 ≈ 空闲 4.04 倍（1 + 3·255/252），路径被迫沿已知区/墙边走，碎块造成之字形 |
| R3 | 局部地面参考“坐”在树冠上 | 看不到树下地面时，格内 10% 分位高度 = 冠底；3183 个旧障碍格的所有点都高于真实地面 1.8 m | 广场、路边出现大片虚假障碍，挤压通道 |
| R4 | 跟随/并行的人、发散建图包无防护 | 0921/0901 包中车后 ~1 m、1.2 m 高重复体素；`diagnostic_replay` z 到 -3398 m、`mid360_20260901_205036` 倾角到 176° 仍被导出 | 轨迹边残留障碍；发散包生成错误地图而无提示 |

## 设计

1. 自车滤波：base_footprint 下足迹外扩 0.10 m、高度 [-0.2, 2.2] m 的柱体内点丢弃。
2. 发散检测：任一关键帧 base 倾角 > 30°，拒绝整包并报告首个异常帧。
3. 持久性：沿用 0.2 m 体素 / ≥2 次 / 跨度 ≥2 帧；另要求体素至少一次在 ≥2.0 m 处被看到
   （MID-360 前倾 13°，静态物在接近过程中必然先被远处看到；只在近场出现的是跟随者/车体附件）。
4. 地面：0.25 m 粗格、10% 分位候选；以轨迹足迹下的 base z 为种子 BFS 生长，
   相邻步高 ≤0.10 m；无数据或上方有结构时平推 ≤1.0 m；遇下坎不跨越，且清掉紧邻下坎的平推格。
   障碍分类参考面在生长区外再平推 2.0 m（只用于分类，不产生空闲）。
5. 分类：|h| ≤ 0.12 m 为地面；0.12～1.80 m 且持久为障碍（车高含杆约 1.75 m）。
6. 射线雕刻：每点从关键帧原点做 2D DDA（≤30 m），射线高度在 [-0.12, 1.0] m 且该格地面已生长
   时计一次穿越；非地面端点前 2 格不雕刻。
7. 判定：占据 = 格内持久命中 ≥2，或格内 ≥1 且 3×3 邻域 ≥2 并来自相隔 ≥2 的关键帧（多孔植被）；
   小连通域剔除 + 1 格闭运算；命中永远优先于穿越，有持久命中的格不会被判为空闲。
   空闲 = 非占据 + 地面已生长 + （地面回波 或 ≥2 次穿越）。
8. 轨迹足迹（外扩 costmap padding 0.03 m）强制空闲，保证示教路径可规划；
   ≤0.5 m² 且四周全为空闲的未知孔填为空闲。其余保持未知。
9. `debug/`：地面来源、射线穿越次数、地面/参考高度（float32）；`metadata.yaml` 增加
   `traversability:` 统计。

## 实现

| 文件 | 说明 |
| --- | --- |
| `src/TraversabilityGridBuilder.cpp`、`include/.../TraversabilityGridBuilder.hpp` | 新模式全部算法 |
| `include/.../ProjectionParameters.hpp` | 新枚举值、`RobotModel`、`TraversabilityParameters`、`TraversabilityStats`；原字段不变 |
| `src/ParameterLoader.cpp` | 读写 `robot:`、`traversability:`；新模式参数校验 |
| `src/main.cpp` | 新模式分派，要求 `--package` |
| `src/OccupancyGridWriter.cpp/.hpp` | 可选参数写入 `traversability:` 统计 |
| `src/PCDProjector.cpp` | 对新模式明确报错（避免 Map Studio 等单点云调用静默退化） |
| `config/projection_traversability.yaml` | Bunker v1 外参/足迹与全部参数 |
| `test/test_traversability.cpp`、`CMakeLists.txt` | 10 个新测试 |

`ProjectionParameters` 结构体布局变化，依赖本库的 `agt_map_studio` 需要一起重编译（已验证）。

## 验证

所有数据可复现，产物在 `~/ros2_ws/experiments/mcp_traversability_20260924/`。
规划评测：Nav2 Humble 1.1.20 planner_server + global costmap（生产参数），
轨迹上随机 40 对起终点（seed 7，10～45 m），另做 8 组×20 步沿轨迹前移的重规划。

### 当前激活地图对应的包（0922 fixed replay，702 关键帧，365.7 m）

| 指标 | 现网地图 | traversability |
| --- | --- | --- |
| 轨迹 ≤1 / ≤2 / ≤5 / ≤10 m 未知 | 2.2% / 6.6% / 28.3% / 51.4% | 0.0% / 3.3% / 17.3% / 33.7% |
| 足迹扫过区域 未知 / 占据 | 1.74% / 1.04% | 0 / 0.004% |
| 轨迹距占据 < 0.43 m | 44.0%（82 段） | 0 |
| 椒盐指数；10 m 内最大空闲连通块占比 | 14.7%；86.0% | 1.45%；99.3% |
| 规划成功率 | 20% | 100% |
| 路径长 / 直线距离 | 1.236 | 1.145 |
| 路径经过未知 | 2.1% | 1.0% |
| 平均净空 | 1.77 m | 2.01 m |
| 转角 rad/m；之字形 次/10 m | 0.263；6.23 | 0.207；3.77 |
| 导出耗时 / 峰值内存 | 1.3～1.8 s / 229 MB | 2.3 s / 288 MB |

### 其他建图包（现网导出器 → traversability）

| 包 | 5 m / 10 m 未知 | 轨迹 < 0.43 m | 规划成功 | 路径经过未知 | 之字形 次/10 m |
| --- | --- | --- | --- | --- | --- |
| full_test_20260921_102643 | 25.8/51.2% → 11.4/27.3% | 36.1% → 0.5% | 45% → 100% | 4.9% → 0 | 7.48 → 3.50 |
| mapping_20260901_211105_decoupled | 23.3/49.8% → 11.3/24.6% | 25.9% → 0.09% | 62.5% → 100% | 3.6% → 0 | 6.91 → 2.31 |
| mid360_20260901_205036_validated_v2 | 25.4/51.1% → 10.6/25.9% | 36.2% → 0.6% | 47.5% → 100% | 5.0% → 0 | 8.23 → 3.01 |
| live_..._diagnostic_replay | — | — | 拒绝：182/878 帧倾角 > 30°（最大 150°） | | |
| mid360_20260901_205036 | — | — | 拒绝：58/396 帧倾角 > 30°（最大 176°） | | |

贴墙比例（净空 < 0.8 m 的路径占比）上升（例如 0921：2% → 22%）：此前不可规划的窄通道
（如 0921 起点走廊两侧 1.1～1.7 m 高的悬挂枝叶，两次经过均被观测到，属静态）现在可通行。

### 安全审计：旧障碍 → 新空闲

激活包中 3875 个“旧图占据、新图空闲”格（已排除足迹扫掠、轨迹 1 m 内与旧图小碎点）：
82% 所有点均高于新参考地面 1.8 m（树冠），8% 格内无点（旧闭运算），7% 为地面点，
1.7% 含零星低空点（共 116 点，均低于占据阈值），0.6% 疑似路沿。
开发中据此修正了两处问题：多孔植被（每 5 cm 格 ≤1 命中、射线从缝隙穿过而被判为空闲）
→ 邻域支撑 + 命中否决；并行行人 → 近场观测规则。

### 消融（激活包，每次去掉一个组件）

| 去掉 | 结果 |
| --- | --- |
| 射线雕刻 | 空闲格 144 万 → 28 万，5 m 未知 17% → 72%，椒盐 59% |
| 足迹扫掠 | 轨迹 < 0.43 m 升至 6.6%，轨迹分成 2 段 |
| 足迹扫掠 + 自车滤波 | 25.5%（51 段），即自车滤波单独贡献 25.5% → 6.6% |
| 邻域支撑 + 命中否决 | 占据少 3.8 万格（植被安全性变差，见审计） |
| 近场规则 | 本包影响很小；0921 类跟随场景用于去除车后残留 |

### 规划器参数对照（v3 地图，生产行为树以外的参考）

| 配置 | 成功 | 长度比 | 转角 | 之字形 | 重规划跳变 p90 | >0.2 m 次数 |
| --- | --- | --- | --- | --- | --- | --- |
| Smac2D ctm 3.0（生产） | 100% | 1.145 | 0.207 | 3.77 | 0.27 m | 23/152 |
| Smac2D ctm 2.0 | 100% | 1.114 | 0.203 | 3.77 | 0.27 m | 23/152 |
| Theta* | 100% | 1.095 | 0.273 | 5.84 | 0.11 m | 8/152 |

建议保持 SmacPlanner2D 与 ctm 3.0。现行行为树只在路径失效时重规划，而全局代价图无障碍层，
重规划跳变基本不会触发；若以后恢复周期重规划，Theta* 可把跳变 p90 降到 0.11 m（拐角更尖）。

## 已知限制与后续

- 2D 图按整车 1.8 m 高度判障：只有车尾中心的杆达到 1.75 m，车身两侧可从更低的枝叶下通过，
  2D 表达无法区分；已用足迹扫掠保证示教路径可规划。
- 地面生长以 0.10 m/0.25 m 步高判定可通行；0.12～0.2 m 路沿可能被当作缓坡（审计中 0.6%）。
- 负障碍（下坎）只保证不外推地面，保持未知，不标为占据。
- 仍只使用约 1/72 的原始点。Phase 2：从原始 bag 按 C_k = T_map_body · T_odom_body⁻¹
  稠密重积分，可进一步减少远处未知。
- 导航仓库 `pcd_to_nav_map` 只吃合并 PCD（无射线、无自车滤波），碰撞带默认上限 1.50 m，
  低于杆顶 1.75 m；如继续使用该转换器，建议至少调高上限并使用轨迹扫掠。
- 未上车验证；地图包需按 `FIELD_MAP_FINALIZATION.md` 做静态重定位与低速试跑后再切换。

## 使用与上线（需人工确认，本次未执行）

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
PKG=~/ros2_ws/experiments/artifacts/output/live_mid360_20260922_143427_fixed_replay/map_package
ros2 run agt_pcd2grid_exporter pcd2grid_exporter --package $PKG \
  --config $(ros2 pkg prefix agt_pcd2grid_exporter)/share/agt_pcd2grid_exporter/config/projection_traversability.yaml \
  --output /tmp/nav_trav
# 复用现网定位资产，新建版本（不覆盖旧版本）
OLD=~/ros2_ws/maps/bunker_mid360/20260922_143427-fixed-replay-v1
ros2 run agt_map_manager create_map_package --map-id bunker_mid360 \
  --map-version 20260922_143427-fixed-replay-trav-v1 \
  --source-pcd $OLD/localization/global_map.pcd --navigation-dir /tmp/nav_trav \
  --relocalization-assets-dir $OLD/localization/relocalization \
  --generation-pipeline /tmp/nav_trav/projection.yaml
ros2 run agt_map_manager select_map_package --map-id bunker_mid360 \
  --map-version 20260922_143427-fixed-replay-trav-v1   # 回退：选回 -fixed-replay-v1
```

`create_map_package` 只复制 `map.yaml`/`map.pgm` 等白名单文件，`debug/` 不会进入正式包。
`review.py --config` 已支持传入该配置。候选导航图已生成：
`experiments/mcp_traversability_20260924/runs/v5_live_mid360_20260922_143427_fixed_replay/`
（范围完整覆盖 `global_map.pcd`）。

## 实验产物

`~/ros2_ws/experiments/mcp_traversability_20260924/`：`tools/`（评测脚本），`runs/`（各版本导出），
`analysis/`（质量、阻塞、规划、差分、审计），`ablation/`，`worktree/`（本分支）。
