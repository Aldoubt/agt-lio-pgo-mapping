# 分支整合记录 — 2026-10-26

本次已把八条保留分支的历史纳入同一代码树，协调真实行为和测试后，合入 `main`，并将远端收敛为 `main` 与 `integration/mapstudio-mapping-product-v1` 两条长期分支。历史已合并不代表每项旧实现都原样启用；下表记录保留、替换及未验证的部分。

## 来源与整合顺序

以下为整合前已核对的远端提交；完整 SHA 保留后续可追溯性，不依赖旧分支名称永久存在。

| 原分支 | 固定提交 | 纳入的能力 |
| --- | --- | --- |
| `main` | `62e80002cf2db12fce8332688dc63f4a94b87f12` | 离线 MapStudio 草稿/研究工具、空间证据及可选依赖边界 |
| `feature/greenhouse-topology-topk-v1` | `4e993c5d33fc7cc110a578103a6b5576a181ca06` | 拓扑审核、不可变分块、单查询 Top-K 证据与显示 |
| `integration/mapstudio-mapping-product-v1` | `11361fca3bcb1e664d45760944621330c8eb0eee` | Study CLI、Query Set、产品配置与 processor 注册表 |
| `feature/greenhouse-coverage-authoring-v1` | `669c63650c3824cdb7c0f714a6a061161e20e798` | 离线全覆盖路线、验证、现有 Route 适配及 Studio 入口 |
| `feature/mapstudio-operator-workflow-v1` | `111db625ab5bd8f3f6caa347ba37800677292e24` | Operator/Study 工作流及显示抽样实现来源 |
| `feature/spatial-map-coverage-geometry-v1` | `62601437c79441d85935585d13f9fc236895645e` | 空间覆盖采样、几何诊断与 Phase 3C 分析 |
| `feature/yhs-field-appliance-v1` | `539352d309deef4f0acfe5f2bd783d5e72623a88` | 挂载标定需求与 YHS 传感器建图入口 |
| `fix/field-map-cleanup-20261024` | `be2d1c16c1c38f878eec903e3ae5cd3da868f2a7` | 时间持久性过滤的静态/移除点云及报告 |

整合前 `topology → integration → coverage` 已是严格祖先链，分别相差 25 和 1 个提交。以 coverage 为起点，依次纳入 geometry、main、Operator、YHS 和 field-cleanup 历史，再做有针对性的修复。main 当时有两个独有提交，不能因数量少就视为已被 coverage 覆盖；其 GUI、依赖和草稿行为必须处理。

原 PR #2 的 base 为 topology、head 为旧 integration，仅代表上述祖先链中间的一段。本轮将其目标和描述更新为完整产品整合，以实际构建和测试记录说明验收范围。旧 [2026-10-09 审计](mapping_product_integration_20261009.md) 保留原样，其未整合状态属于历史记录。

## 最终行为与保留边界

| 模块 | 本轮整合后的行为 | 尚不能据此声称 |
| --- | --- | --- |
| MapStudio | 保留 main 的草稿保存/重开；Study 使用独立 `StudyWorkflowDialog` 复用既有 CLI；保留结构、单查询证据和覆盖路线入口；显示抽样不改分析输入 | 原生构建通过不等于真实显示器上的全流程交互或真实数据验收 |
| Query Set / Study | 手工查询或审核行道采样、冻结版本、批次运行、部分失败/取消、证据失效与导出 | 同会话结果不是独立真值；mock 查询不证明 BBS/GICP 精度 |
| 空间覆盖/几何研究 | benchmark 默认 `--study phase3b`，`--study phase3c` 显式启用扩展候选与诊断，模式进入实验身份 | 研究中的 coverage 不是机器人覆盖路径；不能混用两个模式的结果身份 |
| FAST-LIVO2 标定 | bag/live/YHS 通过 `--backend-profile` 或 launch `backend_profile` 指定完整 profile；启动前冻结实际 native config/profile，导出其原始字节与 hash | 未验证现场外参正确性；不从配置自动推断车体坐标或 TF |
| 地图清理与派生 | 栅格导出以事务方式保存静态/移除点云及报告；手工 refine 在地图坐标执行规则，同步过滤 body-frame patches 并重建 map，保留位姿和其他点字段 | 持久性过滤不等于自动识别人；侧文件不单独成为 source package 或 READY Site/Route |
| Python 标注到覆盖 | 审核行道走廊、方向、边界和选定地头后，导出覆盖结构；非 map 坐标要求显式审核变换；由现有 coverage loader 校验 | 作物行参考线不直接变成可通行路径；自动候选不等于人工批准 |
| 温室路线 | 保留 rows/area、二维净空、整车扫掠、阿克曼约束、漏作统计及现有 Route 合同适配 | reference 后端不是 OpenNav/Smac 替代；规划成功不等于实车可执行 |

旧 YHS 分支的 FAST-LIO2/PGO 启动配置未恢复。旧 `fastlio_config`/`--fastlio-config` 非空输入明确拒绝，替换为 FAST-LIVO2 `backend_profile` 及启动快照。唯一已接受的操作员 session backend 仍是 `fast_livo2_lio`，视觉关闭、无 PGO；YHS 入口只管理传感器建图，不启动底盘或发布 base TF。

手工 refine 使用已有 `refined_mapping_source` manifest/checksum，原 source 不改写。地图删点会同步删除 patch 观测，避免下次栅格重建重新出现；空 patch 保留对应 pose。由 patch 重建可能改变父地图下采样后的点密度。`forbidden_zone` 与删点意图不同，保留轨迹仍可能参与已行驶区域的自由空间投影。

覆盖功能仍要求真实 Profile 参数和运行端合同。缺少已验证标定、非零后轴到 base 偏移、倒车/变速等当前不支持的执行条件保持草稿/阻断原因；不会删段冒充全覆盖。READY 使用既有 V4 权威读取器和内容闭包 digest，发布仍需导出时 `--confirm-review`。

## 可复现入口

```bash
# 产品能力与显式挂载标定；dry-run 不启动 ROS
bash scripts/run_mapping.sh --capabilities
bash scripts/run_mapping.sh /path/to/bag --dry-run \
  --backend-profile /mounted/fast_livo2_lio.yaml

# 从已冻结拓扑导出被审核的通行结构；非 map 坐标显式补充变换及确认
greenhouse_coverage_export --topology /path/to/topology.yaml \
  --map-yaml /path/to/map.yaml --output /path/to/new-structure.geojson

# 离线覆盖请求继续引用地图、上述结构和单一 Profile
agt_coverage_authoring plan --request /path/to/request.yaml --output /path/to/new-plan
```

`greenhouse_coverage_export` 对非 map 输入使用 `--map-from-reference DX DY YAW --confirm-transform-review`，以重复的 `--headland ID` 显式选择已审核地头。CLI 不替用户批准行道。Study 的完整命令见 [Operator workflow contract](../contracts/mapstudio_operator_workflow_v1.md)，覆盖请求及发布见 [coverage authoring](../coverage_authoring.md)。

## 验证记录

以下为整合代码的本地实测。六套 Python 回归使用 Python 3.12；原生构建使用真实 Qt 5.15、PCL 1.15 和 ament-index 运行库，直接编译仓库源文件及原有测试目标，没有替换算法。它是离线原生构建，不是 ROS Humble 的完整 colcon 构建。云端 Python 3.10 结果在发布记录中单列。

| 验证层 | 本轮结果 | 证据与范围 |
| --- | --- | --- |
| 六套 Python 回归 | **585 passed，9 skipped** | artifacts 71；benchmark 102 + 5 skip；bringup 153 + 4 skip；refinement 24；coverage 119；annotation 116。跳过原因是原生定位依赖/测试数据与 ROS launch SDK 缺失 |
| 研究几何与 SE(3) 核心 | **7 passed** | 独立 3D geometry schema 1 项、SE(3) 校验与变换 6 项；未运行依赖历史 run008 的另外 8 项 |
| PCL 导出 CLI | **5 passed** | 实际原生进程验证输出布局、不可覆盖、错误参数和输入保护；refinement 的 24 项中另含真实 PCL 重投影 |
| Python Qt offscreen 与标注→覆盖 bridge | **116 passed，计入上述 annotation** | 真正创建 PyQt5 控件并调用现有 planner；显式批准并选择覆盖端部整车包络的地头后完整规划通过，缺少地头明确失败 |
| 原生 C++ 构建与针对性测试 | **149 项通过，29 个 CTest 目标** | 全量 148 项通过后，最后增加 1 项 Study 源绑定检查，重新构建完整 MapStudio 并复验受影响 4 个目标共 18 项；含 exporter 19 项，未执行原生定位算法 |
| MapStudio 真实窗口草稿验收 | **6 个场景、94 项检查通过** | Xvfb/Mesa 下实际 MainWindow：create 17、reopen 16、corrupt 25、interrupt 18、three_d 10、standalone 8；鼠标编辑、Ctrl+S、进程重开、撤销重做、hash/损坏拒绝、原子保存及独立 2D 会话清除旧 3D 状态 |
| Study Qt 与真实 CLI | **PASS** | Project、手工和审核行道 Query Set、冻结、Study 创建/校验/导出；未执行原生定位时保留 QUEUED，不伪造 summary 或精度结果 |
| V4 所有者合同 | **PASS，计入上述 coverage** | 使用外部所有者原有模块验证，未复制 validator；普通云 CI 缺依赖时明确 skip |
| ROS Humble session、真实温室录包与地图质量 | NOT_RUN（本次整合） | 历史 source package 回放结果不能覆盖本轮标定/启动变化 |
| OpenNav/Smac 原生路线、真实显示器交互、机器人闭环 | NOT_RUN（本次整合） | Python/reference/offscreen/native单元测试不能替代这些验收 |

### 云端 CI 的实际覆盖

[mapping-product-contract.yml](../../.github/workflows/mapping-product-contract.yml) 使用 Ubuntu 22.04 / Python 3.10，安装 coverage 的 `reference,test` 依赖及 PyQt5、Matplotlib、jsonschema，固定使用 Shapely 2 和 rsplan 1.0.10。六套测试分进程运行 `pytest --import-mode=importlib -q -ra`，避免同名测试模块和 ROS stand-in 污染；annotation 使用 `QT_QPA_PLATFORM=offscreen`，JUnit 记录包含 skip 原因。

另单独运行不依赖现场数据的研究 geometry schema 与 SE(3) 核心测试。CI 不在缺少依赖时伪造原生成功：ROS launch SDK、native Top-K、PCL 重投影与 V4 所有者测试按既有条件明确跳过。需要 `rclpy` 的 exporter 生命周期测试、Open3D cross-stage 分析、绝对路径 run008 数据集测试及直接导入外部 V4 的 A0 测试不包含在该 ROS-free job；exporter Python 源码仍做语法编译，其派生数据合同由 artifacts/refinement 测试覆盖。原生 C++ MapStudio/PCL 构建也不由该 job 承担。上传的报告仅说明实际运行的层次。

## 发布与分支收敛

2026-10-26 已完成以下远端操作：

- 整合实现提交：`c0e4ffbe3d6befdb515e5b425e117c0482629a1e`。
- [PR #2](https://github.com/Aldoubt/agt-lio-pgo-mapping/pull/2) 已重定向到 `main` 并通过 merge commit 合并；合并提交为 `db50d2e469fe5d232ab5fab41315df626f29cdff`，源码树与受测整合提交相同。
- 精确 PR head 的 [PR CI](https://github.com/Aldoubt/agt-lio-pgo-mapping/actions/runs/40313636394) 和 [push CI](https://github.com/Aldoubt/agt-lio-pgo-mapping/actions/runs/40313628479) 均成功。Ubuntu 22.04 / Python 3.10 共 **584 passed、17 skipped**，含额外 7 项便携研究测试。相对本地，缺少 PCL 的 1 项和外部 V4 的 7 项明确跳过；其余跳过为 native Top-K 5 项、ROS SDK 4 项。
- 确认上表全部八个来源 tip 均为新 `main` 的祖先；使用每条旧引用的精确 SHA lease，一次原子推送删除六条已吸收的 `feature/*` / `fix/*`，同时将 integration 快进到主线。提交历史保留，未 squash 或改写历史。
- 远端保留 `main` 与 `integration/mapstudio-mapping-product-v1`。本记录的发布补记同步进入两条分支，不改变已验证的生产源码。

后续以 `main` 作为共同基线，跨模块整合进入现有 integration；确需隔离的小功能分支在合并后及时删除。历史分支按上表 SHA 查询，不重新创建长期平行实现。
