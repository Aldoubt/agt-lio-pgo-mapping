# AI task entrypoint — 建图仓库

目的：用最小阅读集进入一个有界任务，避免每轮重读论文、历史聊天与整库。固定仓库 `/home/yangxuan/ros2_ws/src/agt_mapping_framework`；先读根 `AGENTS.md`，再选下表一行。

## 当前目标与状态

多源输入 → 既有关键帧方法分块 → 离线重定位歧义诊断/可视化/环境改造评估 → 导航地图与离线路线编辑 → 既有合同发布给V4。
A0 **PARTIAL**；A1–A7 未因需求更新自动启动。现有建图/编辑/离线分析能力与本轮升级是否完成分开记录。
用户指定默认已落入bag/live操作员workflow：FAST-LIVO2 LIO-only（`fast_livo2_lio`），`img_en=0`，不使用FAST-LIO2、不启动PGO；green-house全量bag source package验收通过。session CLI当前只接受该验收backend，低层registry保留的profile不代表session/export均已支持。分块只需paired patch+pose。
需求唯一入口：[alignment plan](map_task_asset_authoring_alignment_plan.md)；资产合同基线：[A0 acceptance](contracts/a0_authoring_contract_acceptance.md)。本文只导航，不复制字段/hash真值。

## 按任务读取，不全仓扫描

| 选定任务 | 需求章节 / 最小源码阅读集 | 适用验证 |
| --- | --- | --- |
| 补A0 | plan §6 REQ-PUBLISH、§7、§8；A0 acceptance；tests/contract_fixtures/a0 | 现有validator/loader与小规范fixtures；PROPOSED guard不是生产validator |
| 多源输入 | REQ-SOURCE；bringup/.../backend_registry.py、frontend_map_exporter.py；artifacts/.../frontend_package.py、validation.py | 适用source/schema/provenance tests；bare PCD不能伪造trajectory |
| K关键帧分块 | REQ-BLOCK；frontend_package.py paired assets；外部PGO getSubMap只读方法参考；greenhouse.py query/target helpers | LIO poses即可，无PGO运行依赖；先冻结block合同；端点/frames/pose/hash/coverage/确定性 |
| 默认入口维护 | plan用户默认政策与§8迁移验收；backend_selection.yaml、backend_registry.py、cli.py、session_launch.py、live_launch.py、mapping_v0.launch.py | FAST-LIVO2/img_en=0、resolved profile path、FAST-LIO2拒绝、无PGO；改默认须重跑bag/session package验收 |
| 离线查询/歧义 | REQ-ANALYSIS；benchmarks/agt_map_localization_benchmark/{GREENHOUSE.md,setup.py}与greenhouse.py、topk_trace_replay.py、topk_ambiguity_analysis.py | 对应test_*；native integration单独声明依赖，不重跑全建图 |
| evidence浏览/改造 | REQ-VIEW、REQ-INTERVENTION；现有analysis plot、annotation、Studio geometry loader | 真实小fixture/coverage/UNKNOWN/方案与已安装区分 |
| 编辑/派生一致性 | REQ-DERIVE；SelectionManager、RefinementModel；refinement/.../pipeline.py；pcd2grid入口/patch消费者 | A1适用回归：编辑重放/patch一致、unknown意图、undo一次一变更 |
| HMI/Facade/job | REQ-HMI；Studio main.cpp、WorkflowSession、ExternalToolRunner；scripts/map_studio.sh | A0 DTO需先完整；GUI/headless/cancel/restart/stale，mock/real分开 |
| Route | REQ-ROUTE；既有READY Route runtime reader/resolver只读；未来选定authoring adapter | 曲线/平滑/可行性、Map/Profile exact binding、CSV/manifest hashes |
| 正式发布 | REQ-PUBLISH；source writer/validator；legacy Site schema/publisher和V4 audit只读相关章节 | roundtrip、原子/不可覆盖、portable、hash负例、不activate |
| 论文实验 | plan §2、REQ-ANALYSIS/INTERVENTION；paper/GREENHOUSE_PROBLEM_EXISTENCE_STUDY.md、paper/topk/RUN_REPORT.md、EXPERIMENT_VARIABLE_DEFINITION.md | 历史数字不当新实验；新协议/原始trace/独立session/参考等级 |

表中 `...` 是定位提示，不是可执行路径；用 `rg --files` 定位选定模块。只有遇到当前模块的明确依赖，才扩阅读范围。native算法权威在外部navigation/vendor，禁止复制成mapping第二份。

## 一次任务的输入模板

```text
固定仓库：/home/yangxuan/ros2_ws/src/agt_mapping_framework
只执行：<A0/A1/A6子任务 + 一个REQ/具体交付>
先核对 branch/HEAD/git status 与 AGENTS.md。
最小阅读集：<从上表选择文件/符号>
允许修改：<明确路径>
复用：<既有producer/validator/算法/测试>
验收：<正常与适用负例、输出身份/证据>
停止条件：完成本任务即停止，不自动进入下一阶段。
```

## 关键语义速查

- block_keyframe_count/half_range ≠ query_accumulation_frames ≠ candidate_top_k。
- XYZ几何、原始reflectivity、confidence可视化intensity不混用。
- local basin ≠ 无初值GLOBAL正确；native success ≠ offline reference正确 ≠ runtime accepted。
- 同sessionreference不等于独立GT；物理UNKNOWN、NO_DATA、未尝试/timeout保留分母/删失。
- 论文中途global loss保留当前segment；断电cold start可能无row/segment身份，须V4独立验证。
- 同bag可以做配对离线诊断；读REQ-ANALYSIS下SAME-BAG协议。已有centered query含future帧，不能直接当因果cold/loss回放；记录路径恢复条件与自行安全到达分开。
- Site/Route/source/layer/Task digest来源分开；新Route不回写旧Map。
- 当前外部HMI服务、通用block产品、改造建议产品、Route authoring均PROPOSED；CLI存在不等于API已实现。

## 状态/证据交付

每次记录改动文件、实际复用、合同/能力变化、测试命令及PASS/FAIL/NOT_RUN/BLOCKED、未验证项、下一有界任务。
只对本次代码变化运行适用检查；文档调整不自动启动ROS/native/build。历史实验报告不改写成当前实验结果。
论文PDF不放入源码/不重复抽全文；本计划记录研究映射和文件hash即可，只有改论文协议时才读相关节。
