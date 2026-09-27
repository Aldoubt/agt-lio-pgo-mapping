# 空间置信度 Phase 2A / 2B 验收记录

- **日期**：2026-09-27（Asia/Shanghai）
- **仓库**：`/home/yangxuan/ros2_ws/src/agt_mapping_framework`
- **分支**：`refactor/navigation-runtime-v4`
- **范围**：离线查看单 session 观测证据、人工覆盖意图与新 reviewed derivative；**不接导航、不发布 production map、不接传感器或机器人**。

## 1. 交付与职责边界

| 组成 | 所有权与行为 |
| --- | --- |
| 原始 PGO `map_package` | 只读来源；`map.pcd`、patches、poses、manifest/checksums 不因 Phase 2 修改。 |
| 空间 core `agt_spatial_map_core` | confidence 真值、审计字段的 v1 解析与校验、人工 final 值、stable 硬谓词、受校验的源 derivative 读取、分阶段写入新 reviewed derivative、全部输出 checksum。review **不调用**自动证据构建/`calculate_confidence`。 |
| Map Studio `agt_map_studio` | 验证并加载只读体素证据；独立于 raw map 点索引的体素选择；显示 Auto/Final/Observation/Persistence 与 stable **预览**；只在内存中编辑意图、Undo/Redo/Restore Auto；单独保存 v1 YAML；在用户明确确认后调用 core CLI。**不写 confidence PCD、不直接生成 reviewed stable PCD/校验和、不自动发布地图**。 |

`auto_confidence` 是单 session 的重复观测与跨度证据，**不是**长期稳定概率。v1 `geometry_score=1` / `geometry.mode=deferred`；`LOW_GEOMETRY` 仅是人工原因，不能更新该分数。原始 `map.pcd` 点索引绝不可充当 confidence voxel 索引。

提交链：Phase 2A `07b80ab`（只读加载）、`fd9d132`（可视化），额外的俯视相机修正 `b0c581c`；Phase 2B `94547fd`（core review 与审计扩展）、`0c254cb`（Studio 意图编辑与 UI）。本验收文档单独提交；推荐的“2A 两笔 + 2B core、Studio、验收三笔”顺序保留，俯视修正是额外提交。Phase 1 的基础提交结束于 `6fe0081`。

## 2. 使用步骤（操作不混淆）

1. 在 ROS 2 Humble / 对应工作区 overlay 中启动 Studio，先用 **File → Open Mapping / Map Package** 打开同一优化后的 PGO 包，而非导航发布包或任意 PCD。
2. 用 **File → Open Spatial Confidence Derivative** 打开该 PGO 包的五文件 derivative。Loader 验证全量文件 checksum、字段布局、父包 provenance、手动覆盖与 PCD 的一致性、stable PCD 的内容/点数；失败时保持此前模型。
3. 可切换 `Auto Confidence`、`Final Confidence`、`Observation`、`Persistence`、`Show Stable Only`。Ctrl+点击体素看 key/中心/point count/keyframes/首末跨度/三个分数/Auto/来源 final/内存预览 final/原因及 UTC 审计；stable only 只是选择预览，非正式 derivative。
4. 复用原 OpenGL viewer 的 ScreenRect、PolygonPrism、Sphere，以及既有 `SelectionManager` 逻辑；raw 点云和体素各有独立索引/selection 状态。旧 Delete 行为只针对 raw 点，confidence 视图禁用删除。
5. 从 `AUTO` / `FORCE_HIGH` / `FORCE_LOW` / `IGNORE` 选择模式，为非 AUTO 选原因：`PARKING_AREA`、`VEGETATION`、`TEMPORARY_OBJECT`、`CONSTRUCTION`、`MOVING_OBJECT_PRONE`、`LOW_GEOMETRY`、`MANUAL_ANCHOR`、`OTHER`。`FORCE_LOW` 可提供 [0,1] 自定义值。**Apply to selected** 批量修改内存意图；**Undo override / Redo override / Restore Auto** 可回退或取消覆盖，不改原始证据。窗口标题、侧栏和底部状态会显示 DIRTY。
6. **Save Overrides YAML (intent only)** 写入原 PGO 包及源 derivative **之外**的独立 YAML，原子保存；替换既有意图文件需确认，拒绝符号链接和源目录别名。该操作不构建 reviewed map。UTC 时间在 YAML 中加引号以保持跨解析器字符串类型。现有 v1 文件没有可选审计字段仍可读取；可选 `reason` / `edited_at` / `editor` 均先由 core 解析并覆盖测试，其他字段仍拒绝。
7. 只有已保存、未 DIRTY 的意图才能单独点击 **Core rebuild reviewed derivative (new directory)**；用户指定**新或空目录**并再次确认。Studio 校验已加载的源索引与已保存意图摘要，core 再校验源 derivative 四个 checksum 所覆盖文件、父 PGO 包及意图内容，并在同级 staging 写出五文件，发布前重新验证。不能复用/覆盖原包、源 derivative 或非空目标。core 将源人工状态先重置为 AUTO，再从保存的意图应用覆盖：移除旧覆盖时确实 Restore Auto；自动证据原值不重算。输出单独含 `review.source_checksums_sha256`、`review.input_overrides_sha256`、`automatic_evidence: copied_from_verified_source_without_recomputation`；Studio 对完成的文件与 stable 数再次复核，**不自动打开或发布**该 reviewed derivative。

稳定选择为 `final_confidence >= stable_threshold && mode NOT IN (FORCE_LOW, IGNORE)`；没有任何稳定点时拒绝产生伪“有效”的空 stable PCD。`intensity` 是 final 的可视化值，**不是传感器反射率**。v1 PCD key 在 float64 中精确表示范围为 `|index|<=2^53`。

## 3. 隔离构建和回归

实验目录：`E=/home/yangxuan/ros2_ws/experiments/agt_spatial_confidence_phase2b_20260927`。源码仍在目标仓库；`build/install/log` 在仓库外，未污染现有工作区的既有安装目录。复现时：

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
R=/home/yangxuan/ros2_ws/src/agt_mapping_framework
E=/home/yangxuan/ros2_ws/experiments/agt_spatial_confidence_phase2b_20260927
colcon --log-base "$E/log" build \
  --base-paths "$R/core/agt_spatial_map_core" "$R/apps/agt_map_studio" \
  --packages-select agt_spatial_map_core agt_map_studio \
  --build-base "$E/build" --install-base "$E/install" \
  --symlink-install --cmake-args -DBUILD_TESTING=ON
source "$E/install/setup.bash"
export AGT_SPATIAL_DERIVATIVE_TEST_DIR=/home/yangxuan/ros2_ws/experiments/agt_spatial_confidence_20260927/real_derivative
export AGT_SPATIAL_PARENT_TEST_DIR=/home/yangxuan/ros2_ws/experiments/artifacts/output/live_mid360_20260922_143427_fixed_replay/map_package
colcon --log-base "$E/log" test --packages-select agt_spatial_map_core agt_map_studio \
  --build-base "$E/build" --install-base "$E/install"
colcon test-result --test-result-base "$E/build" --verbose
```

**最终结果：两包构建 PASS；89 tests、0 errors、0 failures、0 skipped。**此前只针对 Phase 2A 已提交代码的隔离结果是 68 项，不能冒充 Phase 2B 验收。覆盖 core 可选审计字段的兼容/非法输入、source/parent/output/intent 保护、分阶段源 PCD 篡改拒绝发布、旧时序过滤器硬谓词兼容；Studio Loader 全量校验与真实 PGO derivative、选择模式、编辑器批量 Undo/Redo/Restore/脏状态、分离保存→core review→Loader 复核，以及既有 PCD 读取、旧 Delete、2D/occupancy/forbidden zones/workflow 单测。

## 4. 真实数据与全体素不变量

- 原始包：`/home/yangxuan/ros2_ws/experiments/artifacts/output/live_mid360_20260922_143427_fixed_replay/map_package/`（优化后 PGO）
- Auto 源 derivative：`/home/yangxuan/ros2_ws/experiments/agt_spatial_confidence_20260927/real_derivative/`
- 统计：**702 keyframes，1,546,618 input/usable points，333,839 confidence voxels，Auto stable 123,321**。

### A. 三种原因 / 人工模式的 CLI roundtrip

保存的、带 UTC 审计的独立意图：`$E/real_override_intent_audited.yaml`；`IGNORE`、自定义值 `FORCE_LOW`、`FORCE_HIGH` 共 3 条（未列出条目为 AUTO）。在**新** `$E/real_reviewed_agent_final/` 执行 `ros2 run agt_spatial_map_core agt_spatial_map_export --map-package "$P" --source-derivative "$S" --manual-overrides "$E/real_override_intent_audited.yaml" --output-dir "$E/real_reviewed_agent_final"`：两个父包全量校验均通过；结果 333,839 voxels、123,320 stable、3 overrides；原始包/源 derivative 未变。

### B. 从真实 Studio 图形界面保存并调用 core

在实验目录的 Xvfb + Mesa 软件渲染下实际打开上述原始包与 derivative，使用 ScreenRect 在**体素视图**选择 **1,167** 条、选择 `IGNORE / PARKING_AREA` 并 Apply；stable 预览从 123,321 变为 **122,874**，窗口/底栏显示 DIRTY。另一次 20,098 体素的离线 UI 操作验证了 Apply → Undo → Redo → Restore Auto 的状态/预览回退；这些不是机器人或传感器测试。

在图形界面 **单独**保存 `$E/studio_saved_intent_agent5.yaml`（1,167 条，原因/UTC/编辑者均存在，SHA-256 `6bfa94d971d6cea2feb4dd43bc6e71ca50f863b059a0f8fcb7683bb752efa63d`），随后用户界面按钮确认、调用 core 在**新** `$E/studio_reviewed_agent_final/` 发布 reviewed derivative。Studio 的成功对话框显示“Core created and Studio re-verified”，stable **122,874**，不是发布导航地图。最终 UI 截图在 `$E/ui_stage12_result.png`；`$E/studio_real_roundtrip_verification.json` 是逐体素验算结果。

对全部 **333,839** 个 88 字节 voxel 记录按键比较：源 PCD 的 `[x,y,z,voxel key, point_count, observed/first/last/span, observation, persistence, geometry, auto_confidence]` 与 GUI reviewed PCD **逐字节相同**；自动证据字节流 SHA-256 两边均为 `a452ef999be139e24df40519862a041a9340f1eaefdc06a732b219af810ec7bf`。其余 **332,672** 条完整记录相同；仅已保存的 **1,167** 条键有人工结果变化。依 core stable 硬谓词独立复算值为 **122,874**，与 metadata/stable PCD/Studio 预览一致；reviewed 的四个文件在 `checksums.sha256` 覆盖下全部 PASS。`review.input_overrides_sha256` 精确绑定原始保存意图字节；导出的 manual YAML 会把输入数字以 float32 值规范化（例如 `0.04` 可写成 `0.0399999991`），因此**不以 YAML 文本完全相同冒充数值语义完全不变**。

### 源不可变的可复核摘要

运行前记录的 `$E/real_source_before.sha256` 包含父包 `manifest.yaml`（`2fa22af0d37ea1da7ea4d5f56c07ae62570aa614433ec3fee596a7b718fbcf1e`）、`checksums.sha256`（`6c8a58d4aab8d1342a75d76ca3210793ca21741f3b244e64812f5ee8f43fc853`）、`map.pcd`（`b14492dfea7bc4c4e58e79b74e787c0be5fcc91fd8ec1fe6dc94c8b3fd96e451`）、`poses_timed.txt`，以及源 derivative 的五文件。每次验证后 `sha256sum --check --status "$E/real_source_before.sha256"` 为 **PASS**，core review 前后还执行父包自带全量 artifact validator；原包其他 checksum-covered 文件未被编辑。源 `confidence_voxels.pcd` 的 SHA-256 始终 `381ca1cdee10372856f96437c18a69dbfeb18a9f30e21b8fd7b5a3abbb90591d`。reviewed GUI 输出索引 SHA-256 `13d078b9efd24ed47a01a5837a2b749beca55ca32606dc8da847d273d065fba5`。

## 5. 14 项验收清单

| # | 验收项 | 结果 |
| ---: | --- | --- |
| 1 | 目标仓库/分支/既有工作树保护 | PASS：仅该仓库 core/Studio/本文档；写入前 26 个文件逐 SHA-256 备份在 `experiments/agt_spatial_confidence_phase2_20260927/preexisting_phase2b_backup_20260927_1901/`（tar SHA-256 `395dd7f7ed733ce86d7d8e5516aeab7d7a552cc376b4f892d0cc60075522b76f`）。 |
| 2 | Phase 2A 验证式只读 Loader | PASS：五文件/PCD 结构/父包/稳定结果严格校验，失败保留旧模型。 |
| 3 | Auto/Final/Observation/Persistence | PASS：单 session 证据明确标注，不称长期概率。 |
| 4 | voxel 证据与 stable preview | PASS：702 帧真实样例、阈值与硬排除规则一致，正式结果仅 core 生成。 |
| 5 | 原有三种选择工具与点/体素分隔 | PASS：ScreenRect/PolygonPrism/Sphere 复用，原 raw Delete 无改义。 |
| 6 | AUTO/HIGH/LOW/IGNORE 与自定义低值 | PASS：core 真值，Studio 仅内存预览。 |
| 7 | 原因审计/Phase 1 v1 兼容 | PASS：8 种原因 + 可选 UTC/编辑者先在 core 解析，旧 YAML 可读。 |
| 8 | Undo/Redo/Restore Auto 与 DIRTY 提示 | PASS：批量命令、已保存状态、真实图形界面按钮和状态栏。 |
| 9 | 保存意图和 reviewed rebuild 分离 | PASS：独立 YAML 原子保存；手动确认才调 core，新目录且无导航发布。 |
| 10 | 自动证据、geometry_score、原包与源 derivative 不变 | PASS：全量体素前缀逐字节、checksum 与基线比较；`LOW_GEOMETRY` 不计算几何分数。 |
| 11 | Loader/编辑/选择/roundtrip/core 负例与回归 | PASS：隔离 89/0/0/0；包括分阶段篡改拒绝发布。 |
| 12 | 真实 333,839-voxel 离线 UI smoke | PASS：Xvfb/Mesa 查看、选区、编辑/撤销/恢复、Save、core rebuild 与 Studio 复核。 |
| 13 | 真实 reviewed derivative 校验和及审计 | PASS：3 模式 CLI 与 1,167 条 GUI 意图两条路径；独立逐体素与 stable 数复算。 |
| 14 | 硬件/导航/跨 session 验收 | **NOT_RUN**：下表中的功能被明确排除，不将静态、mock 或离线 UI 当作实机结果。 |

## 6. NOT_RUN / 明确不在本阶段

| 项目 | 结果 |
| --- | --- |
| 真机器人、Mid360/其他实传感器在线运行、控制器或硬件启动 | **NOT_RUN**；本轮只读离线固定 PGO 包。 |
| Nav2、`agt_navigation_v3`、production 地图发布、在线定位/避障、3D-BBS、weighted GICP | **NOT_RUN**；不连接、不修改。 |
| 多 session/change detection、稳定概率统计标定、在线更新、几何特征值/语义过滤、traversability 重构 | **NOT_RUN**；v1 明确单 session + deferred geometry。 |
| 实机连续画线、独立巡检、机械臂/YHS 启停、仿真 | **NOT_RUN**；本轮没有做这类验收，也不声称通过。 |

已提交代码之外的截图、intent fixture、校验脚本、审计输出位于 `experiments/agt_spatial_confidence_phase2b_20260927/`，不会自动进入 production map 或仓库源码提交。**没有修改** FAST-LIO2/PGO 原实现、`agt_navigation_v3`、已有 2D refinement/forbidden zones/TemporalPersistenceFilter 默认逻辑，也没有处理 `agt_robot_hmi/install` 与其他仓库的既有未提交状态。
