# GREENHOUSE TOPOLOGY + TOP-K 交付与 smoke 报告

## STATUS

可复用工具与数据流水线实现完成。真实物理拓扑尚未人工标注，当前 greenhouse 保留 draft/UNKNOWN；不能声称真实物理行指标已验证。用户本轮选择将标注做成可供下次换场景使用的工具。

BRANCH：mapping 与隔离 native worktree 为 `feature/greenhouse-topology-topk-v1`。  
HEAD：mapping `dac9a6142ec58a3dab5e8a5485f3d35ad348e326`；navigation 基线 `4dc92547e9448f675e0da110f6c33d8e0512cc19`。代码修改尚未 Git commit/push。

## EXISTING TOP-K / NATIVE CHANGES

权威源码在 navigation `candidate_bbs_gicp_localizer.cpp` / `polar_context.hpp`。ring distance 先筛选、sector similarity 再排序。native 默认 K=2，wrapper/原 benchmark K=4，prefilter=40；实验 K=10 是显式参数。

新增 opt-in trace、query metadata 和 JSON 依赖。rank/search/score/GICP 数学未修改。mapping 新增只读 replay、enrichment、analysis，不实现 descriptor/BBS/GICP 算法。native 修改 patch 在同目录 `native-instrumentation.patch`。

## SELECTION BEHAVIOUR PARITY: PASS

三个真实 query，每项运行 baseline / trace OFF / trace ON。threads=1、K=2、60 s shared budget、20 s per-candidate budget；所有非耗时 stdout 字段完全一致。descriptor 排序、winner linkage、未尝试删失和错误输出通过检查。

rank_candidates/crop_local_map/downsample/to_eigen 与基线正文逐字一致。实验 native 动态链接到现有 `.agt_native/lib/libcpu_bbs3d.so`、`libsmall_gicp.so`，hash 已记录。此结论限定于测试条件；共享 deadline 临界处不承诺 trace 记账的时间完全一致。

证据：实验目录 `audit/selection_parity.json`、`audit/native_parity_junit.xml`、`audit/final_code_provenance.json`。

## ANNOTATION TOOL

- build：3 个 package 在隔离 build/install 成功；正式 install 未覆盖。
- launch：`./scripts/annotate_map_topology.sh MAP_PACKAGE ANNOTATION_DIR`，也可使用 ROS 包 `greenhouse_annotator`。
- save/load：YAML、GeoJSON、完整 keyframe labels、hash/provenance、内容寻址版本快照。
- undo/edit：顶点拖动、ID/宽度/方向编辑、删除、Undo/Redo；24 个 GUI/core/validator tests 通过。
- validation：ID、几何、NaN/Inf、frame/backend/manifest、scene timestamp-keyframe、corridor overlap、UNKNOWN coverage、跨 backend transform 与 sidecar hashes。
- freeze：人工确认后全文件校验；冻结文件与 sidecars 不可变；新版本使用独立目录。空拓扑不能冻结。

通用命令与 GUI 操作见 [标注工具 README](../../../tools/agt_greenhouse_annotation/README.md) 和 [完整流水线](../GREENHOUSE_TOPOLOGY_TOPK_PIPELINE.md)。

## TOPOLOGY OUTPUT / ROW-RELATIVE STATE

真实输出在实验目录 `annotation/greenhouse_topology.yaml`，0 rows、0 headlands、0 manual scenes。canonical 1180 keyframes 全 UNKNOWN；没有将轨迹分组当作物理行。

`(r,s,d,psi_rel)` 实现及合成几何测试 PASS。s 为沿人工中心线的累计米数，d 为中心线左侧有符号距离，psi 为局部切线相对 heading；超宽度/距离、未确认和重叠歧义保留 UNKNOWN。真实数据尚无物理标注，所以这组变量为 N/A。

Point-LIO 全文件 checksums：1185 文件 PASS。FAST-LIVO2：1167 文件 PASS。真实 GUI 加载 Point-LIO 8,477,666 点并显示 120,000 点，轨迹 1180 keyframes；截图在 `annotation/gui_map_load_smoke_window.png`。

显式 FAST-LIVO2→Point-LIO SE2 由同 bag 时戳拟合，1178 对应点；XY residual median/P95/max = **0.0914 / 0.2842 / 0.4849 m**。这不是物理 ground truth；跨 backend row labels 必须人工检查残差相对行宽的影响。

## TOP-K TRACE: PASS

两种 backend × `MIDDLE_01` / `HEADLAND_01` × 1/3/5 帧，共 12 个 query。每个 N 使用已冻结的独立融合 query PCD，重新调用 native descriptor retrieval；复用既有 heldout maps/assets，不重新建图。

每 query 保存 40 个 ranked descriptor candidates；前 10 个 BBS 均实际尝试，仅 winner 运行 GICP。总计 480 descriptor entries、120 BBS attempts、12 GICP attempts。Top10 内每 query 另 9 个 GICP 未运行，保留 censored。

raw trace 和 enriched trace 分别保存在实验目录 `smoke_replay/<backend>/{native_traces,traces}`。两个 run 的输入 map/query/assets/native/library hashes 在运行前后一致；逐调用命令与结果在 invocations/manifest.json 中。

## METRICS

| 指标 | 实现验证 | 当前真实 greenhouse |
|---|---|---|
| Recall@1/3/5/10 physical row / same-row Δs<1/2/5 m | PASS | N/A：缺人工物理行 |
| WrongRow@K / Top1、known/unknown coverage 与 bounds | PASS | N/A：缺人工物理行 |
| same-row longitudinal median/P95/P>1/2/5 m | PASS | N/A：缺人工物理行 |
| rank-frequency entropy / effective row count | PASS | N/A：无 confirmed candidate rows |
| sector-similarity margin | PASS | 已导出 |
| XY spread / row count / s spread | PASS | XY 可用；row/s 为 N/A |
| BBS basin hit 1 m / 10° proxy | PASS | 两 backend 均 3/6 query 的 Top10 有 observed hit |
| Selected GICP 0.5 m / 5° nominal | PASS | Point-LIO 3/6；FAST-LIVO2 1/6 |
| Runtime Accepted | 接口与删失处理 PASS | N/A：未观察 runtime wrapper policy |

Selected BBS 1 m / 10° proxy 分别 2/6、1/6；native convergence success 分别 4/6、5/6。native success 与离线正确性是不同字段；未尝试 GICP 不填失败。

以上比率是工具 smoke 记录，不是性能排名或新的论文研究结论。每 scene type、每 N、每 backend 只有一个 scene，scene bootstrap CI 为 N/A。物理行 geometry 未确认，不能验证真实错行/纵向歧义假设。

## TABLES / FIGURES

`tables/` 已生成要求的 8 core CSV：

- table_topk_recall_by_scene.csv
- table_wrong_row_by_scene.csv
- table_longitudinal_ambiguity.csv
- table_candidate_entropy.csv
- table_score_margin.csv
- table_bbs_gicp_stage_success.csv
- table_multiframe_topk.csv
- table_cross_backend_topk.csv

另有 query_metrics、candidate_row_coordinates、scene-effect bootstrap 三张补充表。`figures/` 中要求的 7 张 PNG 已生成；未观察的物理指标图明确显示 N/A。`analysis_manifest.json` 记录全部输入、代码与输出 hashes。

## TESTS

最终 Python suite：**83 passed，0 skipped**，包含真实 native parity integration 和 GUI editing tests。隔离 `colcon build` 三 package PASS；最终 `colcon test-result --verbose`：**83 tests，0 errors，0 failures，0 skipped**，原始结果在实验 `audit/colcon_test_result.txt`。

测试覆盖 trace schema/排序/selection parity/default unchanged、读写/编辑/撤销、几何/UNKNOWN、冻结不可变、provenance 篡改拒绝、known-row/longitudinal/entropy/margin、stage censor、多帧配对/scene bootstrap、跨 backend transform 和 7 图渲染。

## NEXT FIELD PROTOCOL

`../NEXT_GREENHOUSE_DATA_COLLECTION_PROTOCOL.md` 给出静态建图 A、停止后主动重定位 B、未来动态遮挡 C；固定 S/N/B/K，独立 restart trials，保留原始传感器与事件。`../EXPERIMENT_VARIABLE_DEFINITION.md` 给出分母、阈值、删失及参考等级。

下一次只需换 map_package/output 路径，人工标注并冻结，再运行 GLOBAL trace 和 analysis。新 map 的 query/descriptor 数据排除与 backend timestamp correspondence 均保留 provenance。未运行 PGO 或完整 greenhouse STANDARD，本轮只运行用户要求的有限原生 GLOBAL smoke。

## REPORT / ARTIFACT ROOT

完整使用流程：`../GREENHOUSE_TOPOLOGY_TOPK_PIPELINE.md`。实现审计：`../TOPK_RELOCALIZATION_AUDIT.md`。

实验根目录：`/home/yangxuan/ros2_ws/experiments/greenhouse_topology_topk_20261003`。旧 paper 输出和 `docs/paper.zip` 未覆盖。
