# 温室 Top-K 重定位实验变量和指标定义

本文件是下一轮数据流水线的指标契约。它定义实验因子、参考等级、分母和缺失规则；本文件不报告新的性能结果。现有单-session problem-existence 数据没有 confirmed physical row ID，不能据此回填真实 WrongRowRate。Topology smoke 中的人为示范标注也必须带示范/待确认状态。

## 1. Independent variables

| 符号 | 字段 | 水平 | 比较约束 |
|---|---|---|---|
| `S` | `scene_state` | ROW_ENTRY、ROW_MIDDLE、ROW_END、HEADLAND | 主比较 ROW_MIDDLE vs HEADLAND，次比较 ENTRY/END；同一 scene ID 和 bag anchor 冻结 |
| `N` | `frames` | 1、3、5 | 同一 anchor 配对，固定积累方向/运动补偿/query frame；各 N 完整重算 descriptor/BBS/GICP |
| `B` | `backend_id` | Point-LIO、FAST-LIVO2 LIO-only | 独立验证 repeatability；通过 stamp 建 scene correspondence，不假定坐标相同 |
| `K` | `candidate_top_k` / `analysis_k` | 1、3、5、10 | descriptor `analysis_k` 可离线截完整 ranking；native `candidate_top_k` 控制 BBS 尝试集合和预算 |

Session/date/growth stage、physical location、restart replicate、initial yaw 和干扰区间保留为设计/分层字段，不把它们隐藏进 `S`。K=10 的一次 native 运行可给 descriptor Recall@1/3/5/10，不能给完整 native pipeline 在 K=1/3/5 的反事实性能。

## 2. Controlled variables

冻结 descriptor 的 rings/sectors/ranges、patch/database 生成与 exclusion 规则、ring-key prefilter 大小；冻结 BBS voxel hierarchy、search ranges、score threshold、逐 candidate/shared timeout；冻结 GICP voxel、max correspondence、target crop/fallback、threads 和 acceptance。同时冻结 raw bag、query selection、时间匹配容差、frame/extrinsic、reference map/拓扑版本和 CPU/ROS 运行条件。

本轮审计基线为 navigation commit `4dc92547`；instrumentation 在隔离的 native worktree 中实现。基线版本不代表旧 install 自动更新，实际二进制和库仍须由 run manifest 的 SHA256 确认。本机既有默认值与显式 benchmark 配置不同：

| 配置来源 | K | Prefilter | 说明 |
|---|---:|---:|---|
| Native CLI source default | 2 | 40 | `candidate_bbs_gicp_localizer.cpp` 中 Options；不能据此推断 wrapper 实际传参 |
| Mapping benchmark `GLOBAL_SETTINGS` | 4 | 40 | 显式参数，paper 模式可覆盖但必须保存 override |
| 本轮 Top-K smoke 计划 | 10 | 40 | 显式 instrumentation 模式；不改变默认 ranking/selection |

参考源代码时 mapping benchmark map/scan leaf 为 0.35 m、asset map leaf/BBS minimum level 为 0.5 m、shared timeout 18 s、per-candidate timeout 8 s。现场实验必须保存完整实际 config 和 assets hashes，不能仅引用本表；源代码更新或 wrapper override 后以冻结 manifest 为准。

`max_row_assignment_distance`、nominal row width、polygon precedence、longitudinal thresholds `[1,2,5] m`、correctness/acceptance thresholds、entropy method、margin epsilon 和 bootstrap 参数均在结果产生前锁定。结果差时不临时更改 K、tau、threshold、voxel 或 query selection；修改产生新 protocol/run ID，旧结果保留。

## 3. Reference status 和 row-relative state

主参考等级为 `MANUAL_TOPOLOGY + SAME_SESSION_FRONTEND_REFERENCE`，不是 absolute GT。人工确认的 row ID 是物理身份标签；frontend XY/yaw 和沿行站点受 LIO 漂移、centerline 绘制和投影误差影响。Independent survey/RTK/control-point reference 作为单独 reference tier，记录其质量和转换，不自动提升全部数据为 ground truth。

对 confirmed row polyline `C_r(s)`，`s` 为按点击顺序累计的平面弧长。对 pose `p=(x,y)` 找最近有效 segment 上投影 `s*`，单位切线 `t=(tx,ty)`：

```text
r       = 人工确认且距离/区划规则允许的 physical row
s       = s* = argmin_s ||p − C_r(s)||
d       = tx * (y − Cy(s*)) − ty * (x − Cx(s*))
psi_rel = wrap(yaw_robot − atan2(ty,tx))
X_row   = (r,s,d,psi_rel)
```

`d>0` 表示按 centerline 方向观察的左侧；反向行驶不改 `s/d` 坐标，`psi_rel` 可接近 ±180°。Polyline 转角、等距离 segment 和重叠 row 的 tie-break/UNKNOWN 行为以冻结 annotation/tool 版本为准，并保留 validator 警告。Scene 位于 headland polygon 时遵循冻结的区划优先级；超出 row assignment 距离、标注不确定或重叠未决时记 UNKNOWN。不能在所有地图点上做无门限最近行分配。

每个 descriptor candidate 的原始 map pose、BBS coarse pose、GICP final pose分别映射为 row-relative state，不能互相替换。`candidate_row/candidate_s` 的 retrieval 指标使用 descriptor database entry 的 map pose；BBS/GICP 的 stage correctness 使用各阶段输出 pose。Descriptor patch 中心不等同于最终 query 位姿，二者的差异分别报告。

Canonical annotation 的 backend/frame 写入 provenance。不同 backend 通过 same bag timestamp 对应 scenes；若 pose 转入 canonical map，保存经核验的 transform、拟合范围与残差。只有 stamp correspondence 而没有坐标转换时，不能把另一 backend XY 直接套入 canonical row geometry；相应 row metrics 标记 `NOT_MAPPED`。

## 4. Candidate ranking 和 score semantics

权威 native 实现先对整个 descriptor database 计算 `ring_key_distance` 并升序选 prefilter，然后对保留集合计算 best sector shift。最终按 `sector_similarity` 降序，近似相等时以 `ring_distance` 升序 tie-break。Trace 要保存最终排序的完整 prefilter 集合，分别标记 native 前 K、BBS attempted 和 GICP attempted。

| 字段 | 语义 | 方向 | 用途 |
|---|---|---|---|
| `descriptor.ring_distance` | ring key 距离 | lower is better | prefilter 与最终 tie-break；不当作 similarity |
| `descriptor.sector_similarity` | sector rotational similarity | higher is better | 最终 descriptor ranking 与 descriptor margin |
| `descriptor.sector_shift/yaw_seed_deg` | 原始 shift 和 yaw seed | 角度约定见 trace | 可视化/审计，不是 correct-pose 标签 |
| `bbs.score` | native BBS best score percentage | higher is better | coarse selection；不可与 descriptor 分数混用 |
| `gicp.fitness/overlap/converged` | native refinement 输出 | 各字段分开 | 收敛与参考正确性/系统接受是不同状态 |

Descriptor score margin 使用完整 ranking 的前两项：

```text
M12 = (S1 − S2) / (abs(S1) + eps),  S = sector_similarity
```

`eps` 固定并写入 analysis manifest（建议 `1e−12`）；缺少两项或非有限 score 为 N/A。它是排序分离程度，不是 calibrated probability。BBS 既有 spatially distinct coarse-seed ambiguity margin 是另一指标，保留其 valid/timeout 条件，不命名为 descriptor M12。Ring distance 的 lower-better margin 如需补充应单列为 `(D2−D1)/(abs(D1)+eps)`，不能混用两种分数。

## 5. Retrieval metrics

设 query `q` 的有序 descriptor candidates 为 `c1...cm`，`T_K={c1...c_min(K,m)}`。每条 query 保存 requested K、ranked count、prefilter/database 大小、K 是否完整和 label coverage。Descriptor returned-no-candidate 是 retrieval failure；trace 缺失/截断或 hash 不符是数据缺失，不能当成已观测 retrieval failure。

### Physical-row Recall@K 与 Pose-region Recall@K

对 row 身份已 confirmed 的 query：

```text
PhysicalRowRecall_q@K = 1[存在 c∈T_K : r_c = r_q]
PoseRegionRecall_q@K(h) = 1[存在 c∈T_K : r_c = r_q 且 |s_c − s_q| < h]
h ∈ {1,2,5} m
```

两者分别回答认对物理行和在该行中找对纵向区域。输出每个 threshold 的表，不能根据数据挑一个最好的 threshold。正确性必须使用 confirmed 标签；UNKNOWN candidate 不是已证实 wrong row。为完整标注集合报告 recall；对带 unknown 的集合同时报告 observed-hit lower bound、有效查询数/标签覆盖，未能判定的非命中不得解释为已证实错误。

对没有 physical row 的 HEADLAND query，这两个 row-conditioned 指标为 N/A。另定义独立指标，按相同 K 图表单列：

```text
HeadlandIDRecall_q@K = 1[存在 c∈T_K : H_c = H_q，且 H_q 已 confirmed]
XYRegionRecall_q@K(h) = 1[存在 c∈T_K : ||p_c − p_q||_XY < h]
h ∈ {1,2,5} m
```

XY-region recall 是对相应 frontend/reference 坐标的相对区域指标；不是 row-recall 的替代定义，也不是绝对精度。Polygon 太大的 H-ID recall可能很宽松，因此两种 headland 定义一起报。ROW_MIDDLE vs HEADLAND 比较要清楚标明所用 correctness definition；不能把 row+Δs 的数值与 headland-ID 直接当同义 Recall。

### WrongRow 和 longitudinal ambiguity

对 confirmed-row query，`Top1WrongRow=1[r_c1 != r_q]` 仅在 top1 row confirmed 时定义。UNKNOWN top1 单列 `Top1UnknownRate`。主 `WrongRowFraction@K` 仅在完整 observed Top-K prefix 且其中每个 candidate 的 physical row 均 confirmed 时定义；存在 UNKNOWN/headland 未赋 row、prefix 截断或query row未确认时为 N/A。设 `L_K` 为 Top-K 中 physical row confirmed 的 candidates：

```text
WrongRowFraction_q@K = count(c∈T_K : r_c != r_q) / |T_K|  # 完整且所有row均confirmed
WrongRowFractionKnownRows_q@K = count(c∈L_K : r_c != r_q) / |L_K|  # 补充条件估计
CandidateRowCoverage_q@K = |L_K| / |T_K|
```

空 `L_K` 的补充条件估计为 N/A。同步保存 known/unknown counts和wrong-row bounds：在完整prefix下，已确认wrong数/Top-K count是lower bound，将所有尚不能判定row的项都计入可能wrong得到upper bound；UNKNOWN并未被判定为wrong。Prefix截断时主比例与其完整K解释仍不可用。补充条件估计不能替代主指标，避免通过删除无法判定 candidates 得到好看的分母。HEADLAND query 没有 row 时 WrongRow 为 N/A；可另报 wrong-headland/unknown distribution。

对 `r_c=r_q` 的 confirmed candidates 计算 `DeltaS=abs(s_c−s_q)`；输出 query 内 median/P95 和 `P(DeltaS>1/2/5 m)`，以及 same-row candidate count。不存在 same-row candidate 时 longitudinal ambiguity 为 N/A，并由 retrieval/wrong-row/coverage指标表现失败；不能填零。总体表先按 query 计算再汇总/cluster CI，额外候选池分位数若展示须注明是描述性池化而非独立样本。

## 6. Candidate entropy 和 spatial spread

默认 `entropy_method=rank_frequency`，每个候选等权。对 Top-K 中 confirmed physical rows 的候选，`p_r = count(row=r)/row_labeled_count`，使用自然对数：

```text
H_row = −sum_r p_r * log(p_r)
effective_row_count = exp(H_row)
```

同时保存 `CandidateRowCoverage@K`、observed distinct row count、UNKNOWN fraction和headland fraction。没有 confirmed row candidate 时 H_row为 N/A；仅一条 row 的 H=0不能掩盖低 label coverage。Row entropy不需要 query 有 row，所以也能描述 HEADLAND query 的 candidate row 分布，但不能推导该 query 的 wrong-row率。

可另保存 assignment-category entropy（所有 `ROW:Rxx`, `HEADLAND:Hxx`, `UNKNOWN` 为独立类别），名称要与 H_row区分；UNKNOWN不是一条物理行。若改用 softmax，必须预先冻结 `tau` 并存 manifest：`w_i=exp((S_i−max(S))/tau)/sum_j exp((S_j−max(S))/tau)`。归一到 confirmed-row子集时需写明，并保留该子集占总权重的 coverage。Score 到概率没有校准；禁止事后选 tau。

Top-K descriptor map poses 的 XY spread为 centroid、每点到centroid距离的 mean/P95、max pairwise distance；输出 point count。Row-relative spread按每一条 confirmed row分别统计 `s` 的范围/分位数，并报 distinct row count；不同 row的 `s=0` 不共享物理基准，不能把所有 row的 s直接混成一个 spread。BBS/GICP pose spread可另报，但不覆盖原 descriptor展布。

## 7. BBS/GICP/Acceptance 的阶段指标和删失

实际 native candidate pipeline对 descriptor前 K按原次序尝试 BBS，受 shared deadline 和 per-candidate timeout限制；选 valid coarse中最高 BBS score，之后仅对 winner运行 GICP。这不是对每个 Top-K candidate都 refine的 pipeline。Trace中未进入 BBS的 descriptor candidates仍需保存。

| Stage | 指标与参考正确性 | 分母/缺失处理 |
|---|---|---|
| Descriptor | `DescriptorRecall@K`；按 row+Δs 或 headland/XY定义 | 所有有效query，另报label coverage与未判定数 |
| BBS | `BBSBasinHit@K`：前K中实际attempted且valid coarse有一个满足XY≤1m、yaw≤10°的参考阈值 | query-level全分母和attempted条件分母均报；deadline未尝试为censored，invalid/timeout单列 |
| GICP | `GICPFinalSuccess`：实际selected winner的final pose满足nominal XY≤0.5m、yaw≤5°参考阈值 | 所有query、GICP-attempted query分别报；converged不等于reference-correct |
| Acceptance | `FinalAcceptedSuccess`：系统明确accepted且reference-correct | 所有启动query/trial；accepted-but-wrong单列，缺acceptance接口为N/A |

`BBSBasinHit`名称在本协议中是 coarse/reference-region命中代理，不保证处于真实GICP吸引域。本轮分析定义为BBS XY≤1m/yaw≤10°、GICP nominal XY≤0.5m/yaw≤5°；将两组thresholds显式写入每个analysis manifest。现场正式实验在run前冻结这些值，保留实际误差，不以BBS `valid=true`替代correctness；后续改变产生新analysis版本。

`gicp.attempted=false` 在非selected candidates上表示native没有运行此阶段，不能记 `converged=false`作为算法失败，也不能从它们构造“any Top-K GICP success”。只有对每个 K重跑现有 native pipeline，才可报告 `GICPFinalSuccess@K` 的pipeline定义；这表示使用K配置时selected-result成功，不是Top-K每项GICP的成功集合。

Trace记录 `attempted=false` 的原因（如outside_native_top_k、shared_deadline、no_valid_coarse、not_selected_for_gicp），以及candidate timeout、query timeout、异常/exit code。右删失、stage not reached、unavailable reference与已观测失败不同。全query operational成功率可把未恢复的有效trial记失败，同时保留stage censor counts；条件指标不能用到达后续阶段的少数query冒充整体成功率。

系统acceptance需来自实际上层接口。Native `ok`/exit code、GICP convergence或离线正确性不自动等于机器人接受；未记录acceptance时只报告`NativePipelineResultAvailable`，`FinalAcceptedSuccess`为N/A。

## 8. Recovery time 和人工介入

RecoveryTime从冻结的`startup_bag_timestamp`计至首个accepted且参考正确的结果，可预先要求持续正确若干秒/输出。保存首次accepted、首次reference-correct和最终稳定恢复时间，防止错误接受掩盖恢复延迟。

Trial超过冻结timeout为右删失；人工介入到发生时间为删失/竞争事件，另报intervention率；录制故障和安全取消单列。成功trial的median/P95只描述条件分布，同时报全部trial数、success/timeout/intervention counts。未恢复时间不能填0或直接排除而不报coverage。若仅离线query，没有真实startup/acceptance/控制事件，RecoveryTime标记NOT_MEASURED，不用native运行耗时冒充实机恢复。

## 9. Scene comparisons、multi-frame 和 cross-backend

主要输出HEADLAND−ROW_MIDDLE的`DeltaRecall`, `DeltaWrongRow`, `DeltaEntropy`, `DeltaScoreMargin`, `DeltaSpatialSpread`与scene-level95% CI；正确性定义不适用时输出N/A及原因。Headland的row-recall/wrong-row不适用时，对双方共同可定义的XY-region recall、candidate entropy/score/spread比较，并独立保留row专属指标。

基本分析单位是scene/query，不是Top-K entries。Query-level原始行保存`session_id, scene_id, location_id, trial_id, query_id, anchor_timestamp, S,N,B,K`。同scene/location中的重复restart、yaw、N、B不是彼此完全独立；bootstrap以scene/location cluster采样，session内分层，多帧与跨backend效应保持anchor配对。固定bootstrap种子、次数和coverage规则；样本很少时CI和结论注明其限制。

Multi-frame必须分别重新生成query descriptor和trace，比较paired `Recall@1/3/5/10`, WrongRow, DeltaS, entropy, margin，以及各阶段attempt/success。不能把LOCAL GICP basin变化直接作为GLOBAL retrieval改善。Cross-backend比较使用相同bag anchor和确认场景；scene correspondence保留keyframe/time offset和坐标映射质量，无法对应的query不强行匹配。

## 10. Paper artifacts 和可复现性

每次analysis保存输入trace/topology/labels/scene/map/config SHA256、代码commit/dirty diff、参数、reference tier、有效分母、缺失/censor原因。Topology freeze后benchmark只读，重新标注生成新版本。建议八张表：

- `table_topk_recall_by_scene.csv`：row与headland/XY definitions、K和1/2/5m thresholds分列。
- `table_wrong_row_by_scene.csv`：wrong fraction/rate、coverage、unknown/headland counts。
- `table_longitudinal_ambiguity.csv`：same-row DeltaS、threshold exceedance、有效query/candidate数。
- `table_candidate_entropy.csv`：method/tau、H_row、effective rows、coverage；category entropy单列。
- `table_score_margin.csv`：score semantics/epsilon、M12、valid count；BBS margin区分。
- `table_bbs_gicp_stage_success.csv`：全query/attempted分母、censor counts、reference correctness和acceptance availability。
- `table_multiframe_topk.csv`：paired anchors、N效应、实际native K、retrieval analysis K。
- `table_cross_backend_topk.csv`：timestamp/coordinate correspondence和paired scene effects。

图表保留指标定义、单位、reference status和CI统计单位。Physical topology示范、UNKNOWN或not-mapped数据的输出必须注明，不能包装成真实physical wrong-row或绝对GT证据。完整现场工作流见[NEXT_GREENHOUSE_DATA_COLLECTION_PROTOCOL.md](NEXT_GREENHOUSE_DATA_COLLECTION_PROTOCOL.md)，native权威行为见[TOPK_RELOCALIZATION_AUDIT.md](TOPK_RELOCALIZATION_AUDIT.md)。
