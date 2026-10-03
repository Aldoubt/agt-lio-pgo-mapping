# GLOBAL Top-K 实现审计

审计日期：2026-10-03。权威实现位于 navigation；mapping 负责数据准备、调用、标注和分析。

## 版本与源码

- navigation 基线：`4dc92547e9448f675e0da110f6c33d8e0512cc19`。
- mapping 基线：`dac9a6142ec58a3dab5e8a5485f3d35ad348e326`。
- 新工作分支：`feature/greenhouse-topology-topk-v1`。
- native 修改在隔离 worktree：`/home/yangxuan/ros2_ws/experiments/greenhouse_topology_topk_20261003/native_source`。
- native 源码：`navigation/localization/agt_global_relocalization_native/src/candidate_bbs_gicp_localizer.cpp`。
- descriptor 权威实现：同包 `include/agt_global_relocalization_native/polar_context.hpp`。
- 审计开始时 navigation 工作区存在开发修改；读取期间被外部工作提交并切换至 `experiment/greenhouse-task-continuity`。隔离基线选用切换后干净的 4dc92547，未覆盖原工作区。

## 已存在的候选链

1. 遍历 descriptor 数据库，计算 ring-key RMS distance（越低越好）。
2. 按 ring distance 排序，保留 `descriptor_prefilter` 个候选。
3. 对此池计算所有 sector shifts 的 cosine similarity，取最大值；相同 similarity 时按 ring distance 排序。sector similarity 越高越好，不能称为 distance。
4. yaw seed 来自候选地图姿态加 sector shift；返回整个 prefilter 排名池。
5. BBS 只尝试排名前 `candidate_top_k`，受共享 deadline 和每候选 timeout 限制。
6. 所有实际尝试且 valid 的 BBS 中选最高 score；并列保持先出现者。
7. **只有这个 BBS winner 运行 GICP**。native success 不能代替运行时 wrapper acceptance。

当前基线会遍历 deadline 内前 K 个候选，高分不提前终止。这是基线中已有的行为。

## 默认参数

| 层 | candidate_top_k | descriptor_prefilter |
|---|---:|---:|
| native CLI 默认 | 2 | 40 |
| 当前 navigation wrapper YAML | 4 | 40 |
| mapping offline benchmark 默认 | 4 | 40 |
| 新 trace replay 实验入口 | 10，明确实验选择 | 沿用冻结配置，通常 40 |

Descriptor 默认参数：20 rings、60 sectors、半径 0.5–35 m、z offset 3 m、max height 30 m。数据库头携带实际参数，trace 记录加载值。

## Instrumentation 修改

新增可选 `--trace-candidates-json PATH`，及 query 的 frames/timestamp/keyframe/scene/backend 元数据参数。未指定 trace 路径时不生成文件、不新增 stdout 日志。

trace 保存完整实际 prefilter 排名池，而不是只保存 winner。每项包括 rank、patch/keyframe、ring distance、sector similarity、sector shift、yaw seed、map pose、BBS attempted/valid/timed_out/score/elapsed/coarse pose、GICP attempted/converged/fitness/overlap/final pose。未运行字段保留 `null`，不伪造失败。

selected 记录 rank、`highest_valid_bbs_score_first_on_tie` 和 native success。文件在计算结束后原子写入；写文件失败只报告 stderr，不改变 native 返回结果。失败返回也保存已观察的 trace。

mapping enrichment 提供同 session reference pose 与 hash provenance。`acceptance.owner=offline_reference_tolerance` 的 0.5 m / 5° nominal 判据是离线参考判据。真实 wrapper acceptance 未观察，必须保留 N/A。

未新增 descriptor/BBS/GICP 算法。`rank_candidates`、`crop_local_map`、`downsample`、`to_eigen` 函数正文与基线逐字一致。rank/score/search/GICP 设置和默认值保持原有值。

## 行为一致性证据

在相同 Release 编译参数和相同动态库下分别构建原始基线和 instrumented binary。使用真实 Point-LIO heldout assets，三种 query：`MIDDLE_01_f1`、`HEADLAND_01_f1`、`MIDDLE_01_f3`。

对每项运行 baseline、instrumentation OFF、instrumentation ON；固定 threads=1、K=2、timeout=60 s、per-candidate timeout=20 s。除 BBS elapsed time 外，**所有 stdout JSON 结果字段完全一致**。验证 trace schema、40 项排序、未尝试删失、winner 与 GICP linkage、错误返回和 trace I/O 失败。7 个 pytest 通过。

证据路径：实验目录 `audit/native_parity_junit.xml`、`audit/selection_parity.json`。正式 navigation 源码/install 未被这些构建覆盖。

一致性结论覆盖上述固定线程和充足 budget 的样例。不能声明所有负载下时间一致：开启 trace 的内存记账会产生小量耗时，共享 deadline 附近候选尝试数仍可能受时间边界影响。正式实验必须冻结线程、预算和运行负载，并记录 attempted/timed_out。

## 分阶段统计约束

- Descriptor Recall@K 可从一次完整 ranked trace 提取 K=1/3/5/10 前缀。
- BBS@K 只对真实观察到的前缀候选计算，保留未尝试及 timeout 删失。
- winner-only GICP 不提供所有候选 GICP@K。不能将未尝试候选视作失败，也不能从 K=10 的一次调用推导改变 K 后的 winner 和 GICP 结果。
- BBS basin hit 使用 1 m / 10° coarse-reference **proxy**，并非运行了独立 basin 验证。
- GICP nominal 使用 0.5 m / 5°，native convergence 另行记录。
- UNKNOWN 行号不算错行；物理行、same-row Δs、entropy 必须报告标签 coverage。

当前温室真实人工物理拓扑尚未确认。轨迹分组不被升级为物理行 ground truth。完整变量定义见 `EXPERIMENT_VARIABLE_DEFINITION.md`。
