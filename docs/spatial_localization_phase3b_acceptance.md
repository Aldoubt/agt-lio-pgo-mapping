# Phase 3B — 独立离线定位 A/B benchmark 验收记录

> **性质：仅离线、仅实验。** 本文的“成功”仅指相对于同一次 PGO 优化位姿的回放误差，不是绝对真值、线上定位改进、机器人安全门限，也不是 `Q = P(success)`。没有发布 production map、`active_map` 或 `map->odom`，没有启动 Nav2、控制机器人或修改运行时定位算法。日期：2026-09-27（Asia/Shanghai）。

## 1. 前置检查与实现归属（OBSERVED）

- Phase 3A 最终提交为 `fd980cadd42082e9c16b7ebf3fb6cf334c306a9e`；工作分支已存在 `feature/spatial-map-localization-benchmark-v1`，其原有首提交 `a82495ad5b37a84a6138b63246abb549d575f830` 的**直接父提交**就是该 Phase 3A HEAD。发现该提交时工作树干净；未重建、重置或覆盖它。修正后的 benchmark 单独提交为 `86b85c8121a36732191d585b7bf227eed8998a28`（直接接在 `a82495a` 后），然后才在干净 HEAD 上启动最终 SMOKE / STANDARD。
- `origin/feature/spatial-map-geometry-evidence-v1` 本地跟踪引用等于 `fd980ca`，引用日志有 `update by push`。此前一次 `git ls-remote` 遇到 HTTP 低速错误，**当时未将其冒称在线验证**；最终重试 `git ls-remote --heads origin` 成功，在线确认 Phase 3A 远端提交为 `fd980cadd42082e9c16b7ebf3fb6cf334c306a9e`，Phase 3B 分支已推送且指向已提交的文档/benchmark。
- 编码前已只读检索整个 `~/ros2_ws/src`：运行中的 LOCAL 来自 `agt_navigation_v3/.../map_gicp_tracker.cpp` 的 small_gicp；GLOBAL 来自 `candidate_bbs_gicp_localizer.cpp` 的 Polar Context + CPU 3D-BBS + small_gicp，资产由 `build_relocalization_assets` 与 `build_relocalization_candidates` 生成。旧的 `agt_relocalization_benchmark` 是参数扫掠工具，不是此次地图变体实验。安装的原生 LOCAL/GLOBAL 二进制和 BBS 动态链接均可用；Phase 3B 只通过原有 CLI 作薄离线调用。
- 当前工作只修改目标 mapping 仓库的隔离 `benchmarks/agt_map_localization_benchmark/` 与本文档。`agt_navigation_v3` 的原有未提交更改，以及 `agt_robot_hmi/install` 的跟踪删除，均未触碰。

### Algorithm Audit（实施前输出的决策）

- **Existing GICP/BBS/descriptor implementation：** LOCAL 的 `map_gicp_tracker.cpp` 和 GLOBAL 的 `candidate_bbs_gicp_localizer.cpp` 均调用实际 runtime small_gicp；后者复用原有 CPU 3D-BBS、`polar_context.hpp` 和两项 native asset builders。external 版本和最终 `.so` SHA 单独记录。
- **Reusable interface：** 原生 `--map/--scan/--pose`、`--assets-dir` CLI 已安装，无需导航 ROS 节点；本包仅做只读 PCD 准备、CLI 参数绑定和结果分析，GLOBAL 无初始姿态。
- **Missing interface：** 原生 JSON 不提供确切 iterations/inliers（为 null）；无已确认的同场地独立 session（Tier 2 NOT_RUN）；Phase 3B 首提交旧阈值和弱方向/XYZ coverage/错误分类不足，已在隔离包修正，未回改 native runtime。
- **Plan：** 保留已存在的 `a82495a`，修复指标与来源锁、合成先行→SMOKE→STANDARD、隔离生成产物并检查来源/回归；不发布候选地图，不改 V1/geometry_v1/PGO/FAST-LIO/Nav2。

## 2. 输入、泄漏和可重现性（OBSERVED）

| 输入/约束 | 固定值及解释 |
|---|---|
| optimized PGO 父 `map.pcd` | SHA-256 `b14492dfea7bc4c4e58e79b74e787c0be5fcc91fd8ec1fe6dc94c8b3fd96e451`；702 个 patch、1,546,618 个点；每个点与 `poses_timed.txt` 的 patch 次序/变换逐点验证。参考只称 `optimized_PGO_pose`。 |
| 冻结 V1 evidence | `confidence_voxels.pcd` SHA-256 `381ca1cdee10372856f96437c18a69dbfeb18a9f30e21b8fd7b5d3abbb90591d`；V1 `geometry_score==1`，原有 stable/deferred 语义不变。原 V1 stable PCD SHA-256 `b1e8ab2a2fbb8360ef40383625239f4559b56dbf9caa8e28ac566e9c4075570b`。 |
| Phase 3A geometry | `geometry_voxels.pcd` SHA-256 `49b6ef2bcfff010f383e9f4e11b86fdc6c5045fec5acaaea6406d89a8d4aa3d5`；仅离线读 Qt、Qr、法线和弱轴，不回写。 |
| 已存在的 Phase 2B reviewed | `confidence_voxels.pcd` SHA-256 `e9dea22639dd1546ca97e2b4d8ae20e1a8dd8e51924f835ad40e3d72d2687c86`，检查其来源哈希、voxel keys 与自动证据复制语义；没有制造 reviewed 数据。 |
| Tier 0 | `SELF_QUERY/DATA_LEAKAGE_EXPECTED`；地图明确包含查询自身 patch，不把高成功率当泛化证据。 |
| Tier 1 | 按固定索引规则（`i%3!=2`、参考点附近 30m、每中心最近不超过 96 个 patch）构造 map；STANDARD 合计 192 个 map patch、中心 175/350 的 ±2 共 10 个查询 patch 全排除。使用参考 pose 圈定 map ROI，存在 **oracle ROI bias**；相同 PGO session 及全 session sidecar 还留下 `pose_graph_leakage_possible: true` 和 evidence-label leakage，远非独立验证。 |
| Tier 2 | 没有经过确认的同场地独立 session；`NOT_RUN`，没有伪造第二次录制或绝对 GT。 |

A 为**父 PGO `map.pcd` 的相应 map-split 原始点子集**，不是会把查询 patch 泄漏进 Tier 1 的完整父图；B 用冻结 AUTO_STABLE voxel 谓词筛选相同原始点；C 用实际存在且验证过的 REVIEWED_STABLE；D 在 B 上只按有效 Qt 的预声明 25/50/75% unique-voxel quantile 筛选。每个 D 各有完全相同输入点数的确定性 `CONTROL_RANDOM` 与 1m XY 网格 round-robin `CONTROL_VOXEL`（seed 20260927），并记录 XY/XYZ 占据、边界和 coverage；这些候选地图**只在独立实验目录**，不进入生产发布路径。此次 STANDARD Tier 1 的 q25/q50/q75 实际 Qt cutoff 为 `0.0028806483 / 0.0504800547 / 0.2033571340`（每个 cutoff 的 eligible 稳定 voxel 群体及来源均见 manifest），并非按 success 后验挑选。

查询按 1/3/5 帧生成：相邻 body patch 以 `T_body_ref_body_i = inv(T_map_body_ref) · T_map_body_i` 对齐后累积。LOCAL 对同一查询/算法/初值在所有地图上复用相同原生参数；STANDARD 的 20 个预声明起点覆盖平移 0/0.25/0.5/1/2m、yaw 0/5/10/20/45° 及组合。GLOBAL 无初值，单独统计。LOCAL/GLOBAL 的原生二进制 SHA、参数、seed、地图/查询/sidecar 输入 SHA、拆分、Qt cutoffs 和候选 PCD SHA 均写入 manifest。原生 `map_gicp_tracker` SHA-256 为 `941dffc4f291a9aa6c5063e30554a2de24868fefc7b40020add7b76238c81e3b`，`candidate_bbs_gicp_localizer` 为 `0e79982e72c5421490777bf5c64d14e1459023b749f5fdb053ee64307df9f816`；builders 的 SHA 与调用参数见 manifest。新 run 还单独哈希动态链接库（small_gicp `.so` `de8cfca931142bb2c00a8b6cda7de568452520a78ef2a2d07b504b619d3ecc97`；CPU BBS `.so` `bcd303a3079a64d22008a006a9cefecd3c4db0bc6e915f9683491419d14bb152`），记录 vendored git commits `2c6c7831...`、`41529a34...`，并给 GLOBAL 生成的所有资产逐文件记 SHA。历史 STANDARD 当时未记录这些库/完整资产哈希，不能追溯性推定。

分析门限固定为 **strict ≤0.2m 且 ≤2°；nominal ≤0.5m 且 ≤5°；loose ≤1m 且 ≤10°**，平移是 3D 距离，yaw 是 map 平面 heading 差；另报告 XY、Z 和完整 SO(3) 角误差，不引入隐藏的 Z 门限。成功要求原生 backend success **且**相应位姿误差过线；fitness/overlap 不能代替姿态误差。原生 CLI 没有返回精确 inlier、iteration 数，两列保持 `null`，MAX_ITERATIONS 仅有原生显式报出时才计。

## 3. 合成与 SMOKE（OBSERVED）

- 原先已完成的 `smoke_20260927_a`：先执行 plane、wall、corner、pole、重复平行墙合成用例；LOCAL 原生 backend 14/15 成功，GLOBAL 2/2；真实 Tier 0/1 共 **216 次 LOCAL + 12 次 GLOBAL**，仅是单 session 回放。原始 run/manifest/JSONL/CSV/图均保留不变。其**旧阈值统计不作为本文最终门限统计**。
- 代码提交 `86b85c8121a36732191d585b7bf227eed8998a28` 后，以新 run-id `smoke_20260927_committed_01` **完成** SMOKE：先合成（LOCAL backend 14/15，GLOBAL 2/2），再真实 Tier 0/1（216 LOCAL + 12 GLOBAL = 228 次）；失败码 `NO_CONVERGENCE=31`，其余类别 0。Tier 1 单帧 5 个起点中，D_QT_q75 strict 0/5、nominal 2/5、loose 2/5，同点数 RANDOM_q75 / VOXEL_q75 都是 5/5；小规模 SMOKE 不替代 STANDARD。manifest `COMPLETED`，其 git commit、输入/代码/linked libs 的前后 SHA 一致，12 组 GLOBAL 资产逐文件记录 SHA。开发期额外的 SMOKE 均独立保留，任何旧 run 不覆盖；合成场景可有精确设计姿态与重叠理想几何，不能外推实机定位精度。

## 4. STANDARD：最终已提交代码的原生试验（OBSERVED）

最终 `standard_20260927_committed_01` 以提交 `86b85c8` 的 **同一输入、benchmark 代码、原生二进制及 linked libraries** 对应的已完成 `smoke_20260927_committed_01` 为前置；真实 Tier 1 地图 192 个 keyframe，预留的 10 个查询 keyframe 与地图不交叉。此次**新执行** LOCAL 1,440 次、GLOBAL 48 次；GLOBAL 的 12 份 descriptor/BBS assets 均逐文件记录 SHA。全部来源、代码和 linked libraries 的前后 SHA 一致，manifest 状态为 `COMPLETED`。这些只是单 session optimized-PGO-pose 参考下的离线观察。

LOCAL backend success **944/1,440**，strict **816/1,440**、nominal **925/1,440**、loose **938/1,440**。 GLOBAL backend success **40/48**，strict **37/48**、nominal **38/48**、loose **38/48**。

Tier 1 LOCAL 每个变体 **120 次**（2 个中心 × 3 个帧数 × 20 个起点）。表中 median 3D 错误只在 native 返回姿态的记录上统计，失败但有姿态时亦保留；成功分子始终按全部 120 次与 backend success 计算。完整 XY/Z/yaw/SO(3) 误差及 median/P95/max、fitness、外部 wall runtime 见该 run 的 `summary.json` 和 `report.md`。

| 地图 | 输入点数 | 占据 XYZ 1m 格 | strict/120 | nominal/120 | loose/120 | 3D error median/P95 (m) | wall median/P95 (ms) | 主要失败码 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| A_RAW | 382,222 | 10,617 | 79 | 87 | 87 | 0.106/1.238 | 1029.9/4305.9 | NO_CONVERGENCE 33 |
| B_AUTO_STABLE | 300,383 | 6,442 | 74 | 89 | 89 | 0.034/1.240 | 588.1/2739.2 | NO_CONVERGENCE 30；WRONG_BASIN 1 |
| C_REVIEWED_STABLE | 300,120 | 6,411 | 74 | 89 | 89 | 0.034/1.240 | 590.9/2732.3 | NO_CONVERGENCE 30；WRONG_BASIN 1 |
| D_QT_q25 | 243,197 | 5,103 | 59 | 59 | 59 | 0.011/2.825 | 236.0/1771.3 | INSUFFICIENT_MAP_POINTS 51；NO_CONVERGENCE 10 |
| CONTROL_RANDOM_q25 | 243,197 | 6,284 | 60 | 64 | 76 | 0.049/1.213 | 716.6/2696.5 | NO_CONVERGENCE 44 |
| CONTROL_VOXEL_q25 | 243,197 | 6,430 | 74 | 89 | 89 | 0.034/1.212 | 572.6/2751.6 | NO_CONVERGENCE 31 |
| D_QT_q50 | 163,852 | 3,914 | 50 | 50 | 50 | 0.035/2.669 | 166.2/2082.1 | INSUFFICIENT_MAP_POINTS 60；NO_CONVERGENCE 10 |
| CONTROL_RANDOM_q50 | 163,852 | 5,956 | 77 | 93 | 93 | 0.025/0.589 | 488.0/2448.8 | NO_CONVERGENCE 27 |
| CONTROL_VOXEL_q50 | 163,852 | 6,380 | 75 | 91 | 92 | 0.039/1.221 | 550.0/2717.6 | NO_CONVERGENCE 28 |
| D_QT_q75 | 75,210 | 2,359 | 20 | 35 | 35 | 0.192/4.628 | 329.5/4831.0 | INSUFFICIENT_MAP_POINTS 60；NO_CONVERGENCE 25 |
| CONTROL_RANDOM_q75 | 75,210 | 5,180 | 92 | 94 | 94 | 0.042/1.458 | 353.5/1579.9 | NO_CONVERGENCE 25；WRONG_BASIN 1 |
| CONTROL_VOXEL_q75 | 75,210 | 6,119 | 82 | 85 | 85 | 0.035/1.312 | 442.0/1769.3 | NO_CONVERGENCE 32；WRONG_BASIN 3 |

同点数 q75 对照：三者均为 **75,210 点**，但 D/RANDOM/VOXEL 的 XYZ 占据分别是 **2,359/5,180/6,119** 格（相对 A 的 **22.2%/48.8%/57.6%**），nominal 分别为 **35/120、94/120、85/120**。单凭相同点数仍未控制空间覆盖；D 的失败不能归因为 Qt 本身，更不能据此自动发布候选地图。

按 case 分类（全体 1,488 次）：`NO_CONVERGENCE=333`，`WRONG_BASIN=6`，`INSUFFICIENT_MAP_POINTS=171`，`FALSE_RELOCALIZATION=2`；其余已定义失败类别本次为 0。`FALSE_RELOCALIZATION` 只表示 GLOBAL backend 成功但**偏离同 session PGO 位姿参考**，不是独立负 session 的假阳性率。原生未导出精确迭代或 inlier 数，对应列均为 `null`；报告了完整 P95/max 和各失败原因。

### 历史 `a82495a` 原生试验的独立复算（补充，非最终新试验）

此前原生 `standard_20260927_a` 在 Tier 1 跑完 **1,440 LOCAL + 48 GLOBAL = 1,488** 次离线定位，随后已有 Phase 1–3A core/Studio 回归记录。由于其第一版报告的成功门限写错，**没有修改这份原生记录**：`reanalysis/standard_a_corrected_20260927_final` 先逐一匹配 immutable 源文件/原生二进制/算法源码 SHA，验证每个 map/query PCD SHA、原始 JSONL SHA、12 个原始 descriptor DB SHA，以及复算原来的 Qt/Qr 数值一致；然后从原始 pose/backend JSON 重新计算本节门限、错误类型和方向指标。新分析 manifest 明写 `POSTPROCESS_NO_NATIVE_RERUN` / `new_native_trials: 0`。因此本节是 **1,488 条旧原生试验的更正后离线分析，不是 1,488 条新试验**。原始 `a82495a` manifest 未逐文件记录其他 BBS coarse assets、linked `.so` 的当时哈希；新分析把**当前**库 SHA 记为描述信息，明确标记 `original_full_bbs_asset_hashes_recorded: false` / `linked_registration_dependencies_original_recorded: false`，不能倒填成原生运行时已验证。上文已完成的新 run 在 manifest 中记录完整 asset 与库 SHA。

补充复核：旧原生数据按修正门限重新分析后，LOCAL 1,440 次与 GLOBAL 48 次的 success 计数、失败 breakdown、各变体占据/点数均与上方**实际新执行的**最终 STANDARD 一致。这是两次独立保留的 run 的交叉检查，**历史复算本身没有新原生试验**；旧实验缺少当时完整 linked-library/所有 BBS 资产 SHA，不能用当前哈希回填其来源。即使有 fitness 的 backend-failed case，也不能用 fitness 替代位姿误差。详细旧实验分布另见历史复算目录的 `summary.json` / `report.md`。

## 5. Qt/Qr 与弱方向（ANALYSIS；不是概率）

`Qr_mapping_view_median` 是 mapping-era sidecar 的局部 per-voxel 值中位数；`Qr_query_conditioned` 则固定查询参考 body 原点，以候选地图点和冻结法线重算局部 Hr，**不会写回 sidecar**。mapping-view 每 voxel 的方向另作符号不敏感的 axial consensus（散乱时 invalid），不能与 query-view 局部谱/弱轴等同。Ht 无量纲，Hr 为 m²，分开算。最终已提交代码的 STANDARD 将 20 个起点按 tier/地图/中心/帧数合组后，Spearman(Q vs 观察到的 nominal fraction)：

| 指标 | 共享 session 的地图/查询组数 | Spearman ρ |
|---|---:|---:|
| Qt_mapping_view_median | 72 | 0.248 |
| Qr_mapping_view_median | 72 | 0.284 |
| Qt_query_local | 72 | 0.248 |
| Qr_query_conditioned | 72 | 0.563 |

这些组使用相同 session、重叠地图、共享查询及多次扰动，**不是独立同分布样本**；Qt mapping 分位 bins 的 nominal hits/trials 为 188/360、237/360、307/360、193/360，亦非单调递增。bins 仅是经验分箱，**不提供校准过的成功概率**。每 case 还给出相对于 mapping/query 两类平移弱轴和旋转弱轴的扰动夹角及绝对弱轴误差恢复量；零扰动、不明确 consensus 和缺失 pose 为 null。例如 B 的 1 帧 LOCAL、平移与 query 弱轴对齐组为 nominal 8/10、中位投影恢复 +0.970m；较强方向组 14/20、中位 -0.038m。此为方向性诊断，不证明因果。

## Hypotheses（尚未验证）

- 在固定点数下，高 Qt 候选可能偏向几何丰富但空间稀疏的区域，令 native 初值中心裁剪后的地图不足 1,000 点；更独立的空间覆盖匹配和同场地新 session 才能辨别 Qt 与 coverage 的作用，**本实验未证明因果关系**。
- Qr_query_conditioned 与观察到的 nominal fraction 的相关性可能含参考原点、查询位置与全 session 法线证据共同作用；**既没有独立校准集，也没有成功概率模型**。

## 6. 测试、完整性与 NOT_RUN

- package 合约/拆分/PCD/Qt-Qr/失败码/门限/弱轴/原生资产 SHA/SMOKE→STANDARD 源码锁单元测试：直接 pytest **14 passed**；隔离 colcon build 成功，colcon test-result **14 tests, 0 errors, 0 failures, 0 skipped**。最终只读验收脚本 `final_verification.log` **退出码 0**：核对已提交代码的 228 + 1,488 条原生记录、全部候选/查询 PCD 的 SHA、每组 GLOBAL asset 的逐文件 SHA、manifest predecessor SHA、前后输入/代码/linked libs 哈希、CSV/JSON 总数/失败分类/门限和 NOT_RUN 标记。原生方法、mapping core 和 Studio 未改动。
- 最终提交后**重新隔离**构建 core + Studio，并运行 `final_core_studio_regression_01/test_result.log`：**112 tests, 0 errors, 0 failures, 0 skipped**；`source_before.sha256` 的 9 个原始来源文件逐项核验成功，V1、reviewed 和 Phase 3A 三组 sidecar 的合计 10 个 SHA 均成功。首次尝试的回归启动脚本由于 ROS setup 与 shell `nounset` 冲突，在编译前失败，日志保留在 `attempts/final_regression_attempt1.log`；修正的仅是实验目录脚本，第二次回归退出码 0。Phase 3B git diff 范围只包含隔离 benchmark 和本文档；本次未修改 `agt_navigation_v3` 的已有未提交变更或 `agt_robot_hmi`。
- `NOT_RUN`：Tier 2（无经证实的同场地独立 session）；FULL（未明确请求）；线上机器人、Nav2、`map->odom` 发布、Guardian/product map gate、绝对真值/独立负 session 假阳性率。不将其写成“通过”。

## 7. 路径与复现

- 程序：`benchmarks/agt_map_localization_benchmark/`，详见其中 `README.md`；命令含 SMOKE → STANDARD 顺序约束，以及离线事后复算工具 `python3 -m agt_map_localization_benchmark.reanalyse`。
- 原生实验（保留）：`/home/yangxuan/ros2_ws/experiments/agt_map_localization_phase3b_20260927/runs/{smoke_20260927_a,standard_20260927_a}/`。
- 更正指标（保留原 native JSONL 且新目录独立）：`/home/yangxuan/ros2_ws/experiments/agt_map_localization_phase3b_20260927/reanalysis/standard_a_corrected_20260927_final/`；内含 `manifest.json`、`results.jsonl`、`results.csv`、`summary.json`、`report.md` 和 `plots/`。
- 已提交代码 SMOKE → STANDARD：`/home/yangxuan/ros2_ws/experiments/agt_map_localization_phase3b_20260927/runs/{smoke_20260927_committed_01,standard_20260927_committed_01}/`（完成状态以各目录内 manifest 为准）。
