# Phase 3A · 独立 Geometry Evidence Estimator 验收记录

- 日期：2026-09-27（Asia/Shanghai）
- 仓库：`/home/yangxuan/ros2_ws/src/agt_mapping_framework`，GitHub `Aldoubt/agt-lio-pgo-mapping`
- 分支：`feature/spatial-map-geometry-evidence-v1`，从已冻结的 `feature/spatial-map-confidence-v1` / `3361962` 创建，创建前工作树干净。
- 范围：**单次优化后 PGO、离线 core 估计、独立 geometry sidecar、Studio 只读查看**。没有运行机器人、传感器、Nav2 定位，也没有发布生产地图。

## 1. 职责、冻结边界与审计

开始编码前只读审计 `core/agt_spatial_map_core/`、`apps/agt_map_studio/`、`exporters/agt_pcd2grid_exporter/` 与 Phase 2 验收文档，并先在对话中输出 implementation plan。审计快照在仓库外 `$E/audit_source_HEAD3361962.tar.gz`，SHA-256 `6741ddac710edbbf7383995bd5b6ec1150f4c08c30bb57cb232fbafd6b0cf4cf`。**未改 FAST-LIO2、PGO、Nav2、`agt_navigation_v3`、3D-BBS、legacy TemporalPersistenceFilter/traversability，亦未把 geometry 反馈给定位或置信度。**

V1 的 `schema_version=1`、`artifact_type=spatial_confidence_v1`、`geometry.mode=deferred`、`geometry_score=1`、auto/final/stable 逻辑和五文件输出均保持原样。Geometry 使用另一个 `artifact_type=spatial_geometry_evidence_v1`，三个新文件由 core 独占生成和验证；Studio 只有 Model/Loader/颜色/inspector，不可修改 evidence、直接生成 stable/map 或自动发布导航包。Phase 2 的原始 `map.pcd` **点索引**与 confidence **体素索引**继续分离，geometry 由 `VoxelKey` 映射到现有体素索引。

六笔提交（每笔从上述基线顺序提交）：

1. `5621691` — unit-aware raw-point PCA 与双 observability 矩阵估计。
2. `ba3fe83` — 独立三文件 sidecar、严格 provenance/CLI 与原子发布。
3. `9fcec90` — 合成场景、数值协变、权重与 core 负向安全测试。
4. `d03c172` — Studio 只读 Model/Loader、5 种颜色与 inspector 最小接入。
5. `f7c0c68` — Studio 颜色、体素索引和真实来源绑定测试。
6. 本文档及 core/Studio README（提交后的哈希以 `git log -6` 为准）。

## 2. 输入、方法和无效值

真实输入：

- 优化 PGO 父包：`P=/home/yangxuan/ros2_ws/experiments/artifacts/output/live_mid360_20260922_143427_fixed_replay/map_package`
- V1 confidence：`S=/home/yangxuan/ros2_ws/experiments/agt_spatial_confidence_20260927/real_derivative`
- 隔离实验：`E=/home/yangxuan/ros2_ws/experiments/agt_spatial_geometry_phase3a_20260927`
- 独立输出：`$E/real_geometry_evidence/`，**仅** `geometry_voxels.pcd`、`geometry_metadata.yaml`、`checksums.sha256`，不会向 P/S 写文件。

由 V1 读取 **float32 `voxel_size=0.200000003`** 与所有 VoxelKey/centroid。像 Phase 1 一样以正规化优化姿态的 `T_map_body` 将 body-frame 原始 patch 点映射为 `p_map`，保留**每条观测自身**的 `t_body_map=T_map_body.translation()`（未假定 `base_link`、TF、URDF）。目标体素中心的 `normal_radius` 原始地图点邻域作**双精度两遍**均值/总体协方差 PCA，按 `min/mid/max` 保存三个 m² 特征值；法向是 min 特征向量（符号无意义）；形状 `linearity=(max-mid)/max`、`planarity=(mid-min)/max`、`scattering=min/max`。少于 `normal_min_points`、共线（`lambda_mid <= normal_covariance_m2`）或无有效数值时 `normal_valid=0`、相关浮点字段 NaN。

对每个目标体素，半径内每个 **有效法向的邻居体素等权**：

```text
Ht = Σ_voxel n nᵀ                                          （无量纲）
g(p) = (p_map - t_body_map) × n                            （m）
Hr = Σ_voxel mean_{该体素原始观测}( g(p) g(p)ᵀ )            （m²）
Q_t/r = 3 λ_min / (trace + 对应单位的 epsilon)
condition_t/r = λ_max / max(λ_min, 对应单位的 epsilon)
```

分别保存 Ht/Hr 三特征值 `min/mid/max`、弱方向（map-frame 单位向量，符号任意）、`Q_t/Q_r`、condition、有效位，以及 PCA 原始邻居点数、有效法向**体素数**、贡献原始观测数。**min_valid_normals 不足或 trace≈0 => 矩阵 invalid、所有其浮点指标 NaN**；不是给 0 或伪“好分数”。没有未作单位处理的 6×6 混合分数，也**没有 geometry_score / confidence_v2**。Q 只度量方向丰富性，并非点密度、长期稳定概率或定位精度。

严格配置 `config/spatial_geometry_evidence.yaml`：PCA `radius=0.4 m, min_points=8`；方向邻域 `radius=0.8 m, min_valid_normals=6`；`epsilon.normal_covariance_m2=1e-6 m²`、`epsilon.translation=1e-6`（无量纲）、`epsilon.rotation_m2=1e-6 m²`。**真实数据未事后调阈值**。

对连续几何，点/观测原点同时刚体变换时，法向、Ht/Hr 与特征谱应协变/不变；点重排只允许浮点舍入差异。固定原点的 **floor 体素网格**和有限邻域在*非整格平移或任意倾斜旋转*下会改变键归属/半径边界，故不能虚称逐体素完全协变。测试对整格平移与坐标轴 90° 旋转验证逐体素数值协变，亦覆盖原始点重排；该离散化限制不靠放宽真实阈值掩盖。

## 3. Provenance / 发布和回读

metadata 绑定 `frame_id=map`、体素尺寸、sorted VoxelKey、P 的 `manifest.yaml` SHA-256 `2fa22af0d37ea1da7ea4d5f56c07ae62570aa614433ec3fee596a7b718fbcf1e`、P 的 `checksums.sha256` SHA-256 `6c8a58d4aab8d1342a75d76ca3210793ca21741f3b244e64812f5ee8f43fc853`，以及 **S 的 `checksums.sha256` SHA-256** `2b8a291f19c03b9442a0f9429ceff7c3ab02c8a5d713e4a4c11d7b5b99d038f5`。core 检查 V1 全五文件、metadata V1 deferred 契约、PCD key/centroid/count、三文件完整 checksum/字段类型/无效 NaN；CLI 在估计前及发布前调用已有 optimized PGO 包 validator，不接受无 patch 的导航包。输出必须在 P/S 外的**新或空目录**；拒绝 symlink、非空/重叠目的地；同级 staging 写完并复核来源后 `rename` 原子发布。Checksum/digest 证明确定的字节来源，**不是有敌手同时控制所有文件时的数字签名**。

```bash
P=/home/yangxuan/ros2_ws/experiments/artifacts/output/live_mid360_20260922_143427_fixed_replay/map_package
S=/home/yangxuan/ros2_ws/experiments/agt_spatial_confidence_20260927/real_derivative
E=/home/yangxuan/ros2_ws/experiments/agt_spatial_geometry_phase3a_20260927
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
source "$E/install/setup.bash"  # 隔离构建的 overlay
ros2 run agt_spatial_map_core agt_spatial_geometry_estimate \
  --map-package "$P" --confidence-source "$S" \
  --config /home/yangxuan/ros2_ws/src/agt_mapping_framework/core/agt_spatial_map_core/config/spatial_geometry_evidence.yaml \
  --output-dir "$E/real_geometry_evidence"
```

实际输出摘要（`$E/real_estimate.log`）：**702 keyframes / 1,546,618 usable raw points / 333,839 voxel keys**；valid normal **300,607**（无效 33,232：全部邻域原始点少于 8）、valid Ht **316,054**、valid Hr **316,054**（无效 17,785：有效法向体素少于 6）。约 **5.81 s wall / 5.09 s user / 最大 RSS 351,404 KiB**；这只是该台机器此样本的离线运行，不是实时性能保证。`geometry_voxels.pcd` 大约 51 MiB。实际 sidecar SHA-256：PCD `49b6ef2bcfff010f383e9f4e11b86fdc6c5045fec5acaaea6406d89a8d4aa3d5`、metadata `7845bc7fd51b607f880380b8bcf7c1cff6d2ce9bee913874c697876fe1935b8a`、checksum 索引 `ce754cdc97888b800a8b66a01b1e871450b1d0c33794ecac5537f1ffd1f69e39`。

`$E/real_distribution.json` 是独立用 numpy 按 PCD 字段解析的**全 333,839 行**复核（PCL binary writer 的页末补零不作为点记录）；没有删除无效行或筛掉尾部。分位数均只对**对应 valid 项**统计：

| 字段 | p05 | p50 | p95 |
| --- | ---: | ---: | ---: |
| 原始 PCA 支持点 | 10 | 45 | 372 |
| 方向邻居有效法向体素 | 17 | 62 | 124 |
| 贡献原始观测数 | 25 | 185 | 1,627 |
| linearity | 0.0806 | 0.3468 | 0.7224 |
| planarity | 0.1164 | 0.4975 | 0.8778 |
| scattering | 0.00444 | 0.0881 | 0.4126 |
| Qt（无量纲） | 0.000274 | 0.0851 | 0.5664 |
| Qr（Hr 为 m²） | 0.000363 | 0.1009 | 0.4837 |
| Ht condition | 2.74 | 28.52 | 10,908 |
| Hr condition | 3.28 | 21.41 | 7,103 |

低 Q/高 condition 与无效邻域真实保留，**没有为了“分布更好”调阈值**；无效 Q/特征值为 NaN，不能混进上述分位数。Source V1 stable 仍恰为 **123,321**。

## 4. 测试、安全和 UI smoke

所有编译输出在 `$E/build`、`$E/install`、`$E/log`，不覆盖已有工作区安装。源码在 feature 分支；先 source ROS Humble / 工作区 overlay，再用 `colcon --base-paths <core> <studio> ... --build-base "$E/build" --install-base "$E/install" --symlink-install --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release` 编译和测试。完整日志 `$E/final_build.log`、`$E/final_test_run.log`、`$E/final_test_result.log`：**112 tests、0 errors、0 failures、0 skipped**（此前 Phase 2 冻结基线 89 项）。含原有 core/Studio/legacy/Phase2 regression，新增：ground / wall / corner / pole / sparse / 共线、每次 body 原点、整格刚体变换/重排、单体素重复点不改变 Ht 体素投票、真实 patch→V1 键匹配、无效 NaN PCD 往返、V1 三文件来源强绑定、checksum 重新计算后恶意字段/metadata 仍拒绝、staging 期间 P 或 S 变更拒绝发布、目标目录保护；Studio 键索引映射、5 色 invalid 和真实 333,839 行只读 Loader 绑定 / 失败保留旧模型。

`$E/negative_cli.log`：实测 CLI **5/5 拒绝**非空输出、symlink 输出、错误配置、非 PGO 输入、把 geometry sidecar 冒作 confidence 来源，退出码均为 1，未发布目标。真实 sidecar `sha256sum --check checksums.sha256` PASS；P 的完整 validator PASS。`$E/source_before.sha256` 在估计、CLI 负向测试、Studio smoke 与最终回归后逐文件验证 PASS；尤其父 `map.pcd` SHA `b14492dfea7bc4c4e58e79b74e787c0be5fcc91fd8ec1fe6dc94c8b3fd96e451`、V1 `stable_map.pcd` SHA `b1e8ab2a2fbb8360ef40383625239f4559b56dbf9caa8e28ac566e9c4075570b` 不变。独立 numpy 逐行对比 **333,839 个 geometry/V1 键、centroid 均完全一致**、V1 全部 `geometry_score==1`。

真实 Studio 在仓库外 Xvfb + Mesa/Qt xcb 打开同一 P、S、G，五个 View action 均实际切换，Inspector Ctrl+click 选择 **1** 个体素显示 normal/PCA min-mid-max、shape、Ht/Hr 各自 min-mid-max/weak/Q/condition/support；无效项显示 INVALID/灰色；stable 预览仍 **123,321**，窗口无 DIRTY。`$E/ui_geometry_five_modes.png`、`$E/ui_geometry_mode_ht_q.png`、`$E/ui_geometry_mode_hr_weak.png`、`$E/ui_geometry_inspector_details.png` 是当次图形证据；UI 的 Publish Workflow 仍是独立旧功能，不因 geometry 自动调用。`$E/ui_phase3a.sh` 为烟测脚本；Studio 无 evidence 编辑/输出入口。

## 5. 未运行 / 下一阶段

| 项目 | 状态 |
| --- | --- |
| 机器人、传感器在线采集、FAST-LIO2/PGO 重跑、Nav2 实机导航 | **NOT_RUN** |
| 3D-BBS、weighted GICP、localization A/B 误差与速度比较 | **NOT_RUN** |
| 多 session、change detection、语义过滤、在线地图更新 | **NOT_RUN（不在 Phase 3A 范围）** |
| 基于 geometry 生成 confidence_v2 或改 V1 stable/production map | **NOT_RUN（禁止在 Phase 3A 声称）** |

Phase 3B 如需实验定位 A/B 或讨论经单位归一化后的 6DoF 融合，必须单独设计离线实验及新版本合同；本侧证链**不证明定位改善**。固定 voxel 栅格对任意 SE(3) 的逐键完全协变不可达，若下阶段需严格连续协变，应明确改用连续空间锚点/重采样定义并另做验证，而不是默默重命名现有 Q。
