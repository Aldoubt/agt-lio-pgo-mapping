# AGENTS.md

适用范围：本仓库。用户明确指令优先。

## 入口与目标
- 固定工作仓库：`/home/yangxuan/ros2_ws/src/agt_mapping_framework`。
- 每轮先报告/核对 branch、HEAD、`git status --short`；保留用户修改，不使用reset/clean/stash清理。
- 先读 `docs/ai_task_entrypoint.md`，选一个任务及最小阅读集；总体需求见 `docs/map_task_asset_authoring_alignment_plan.md`。仅合同任务需完整相关A0基线。
- 不默认重读论文全文、历史审计、聊天或扫描所有模块。当前runtime/interface/test优先于旧占位文档。
- 一轮只做明确授权模块；完成验收即停，不自动进入下一阶段。新增需求不是业务实现完成。

## 实现与证据
- 主线：多源输入、既有关键帧分块方法、离线查询/歧义可视化/环境改造评估、导航地图与Route编辑、既有合同发布。
- 用户指定默认FAST-LIVO2的`fast_livo2_lio`/LIO-only，禁用视觉；不使用FAST-LIO2，不fallback，不强制PGO。操作员bag/live CLI与`mapping_v0`默认已迁移；session workflow当前仅接受该已验收backend，低层registry仍保留其它显式profile。不要恢复旧FAST-LIO2默认。
- 分块使用paired patch+pose，LIO位姿即可；PGO代码出处不等于PGO运行依赖。同bag可做cold/global-loss离线配对诊断，但需明确状态重置/保留、因果窗口和数据隔离，不能代替运动闭环证明。
- 优先Reuse/Migrate/Adapt/Extend；不复制LIO/PGO/descriptor/BBS/GICP算法。
- 复用Site 1.0、READY Route、source manifest/checksum；不另建Map Bundle v1、Route v1、Robot Profile schema或第二套hash。
- block关键帧数、query累积帧数、candidate_top_k分别命名。裸PCD不伪造trajectory/keyframes。
- 不把相似度/收敛/同sessionreference当真实正确性；保留UNKNOWN、NO_DATA、coverage、timeout/unattempted删失。
- source/草稿/profile/config改变，关联preview/review/派生结果按合同失效；旧发布revision不改。
- PROPOSED接口和mock明确标注；规范测试载体不是第二个生产asset validator。

## 跨仓库边界与验证
- 外部历史仓库仅只读；发现dirty标 `DIRTY — DO NOT TOUCH`。本次授权mapping工作不自动授权外部源码修改。
- mapping生产资产；V4拥有在线定位/中心线跟踪、Motion Guard、任务执行、TF与active map。无授权不启动机器人动作或修改runtime active state。
- 文档任务只做文档检查；业务任务运行对应有意义的测试，报告PASS/FAIL/NOT_RUN/BLOCKED和未验证项。
