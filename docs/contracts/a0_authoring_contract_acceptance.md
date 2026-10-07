# A0 — Authoring contract 冻结与验收记录

日期：2026-10-06。固定建图仓库：`/home/yangxuan/ros2_ws/src/agt_mapping_framework`。

## 状态

**STATUS: PARTIAL**。身份 / digest 对照、operation envelope 与 scenario fixtures 已建立可审阅基线，但完整 operation DTO/调用边界、最终 checksum coverage 决策及若干可执行规范负例仍未闭合。2026-10-06 复核并按用户新目标调整需求：多源输入、关键帧分块、离线查询/证据/改造评估及外部 HMI 的新增子合同尚待冻结；不能称全部接口已冻结。

生产 AuthoringFacade、mapping-owned Site/Route publisher、Site→Route resolver 未实现属于后续阶段的 NOT IMPLEMENTED，不要求 A0 提前开发它们。A0 应指定唯一推荐 owner、冻结规则，并用明确标为 PROPOSED 的小型规范测试载体验证样本；该载体不是第二个 authoritative asset validator。本记录保留原有测试观察，不把规范样本 PASS 描述成生产集成 PASS。

这是 A0 的结束记录，不授权 A1，也不尝试以业务修改弥补缺口。V4 边界依据只读的 [Contract Reuse Audit](/home/yangxuan/ros2_ws/src/agt_navigation_v4/docs/migration/contract_reuse_audit.md) 与 [Phase 1 Capability Boundary](/home/yangxuan/ros2_ws/src/agt_navigation_v4/docs/migration/phase1_capability_boundary.md)。本文中的 `RECOMMENDED` 是后续唯一 owner 决策；`IMPLEMENTED` 只表示当前源码存在，测试通过情况另列。

## Git / 保护范围基线

| 仓库 | Branch / HEAD | 本轮观察到的工作区状态 | 本轮处理 |
| --- | --- | --- | --- |
| mapping（固定仓库） | `feature/greenhouse-topology-topk-v1` / `214a7f50a22e28a01c8e5ecf16ff9bbcb937d56b` | 初始仅有上轮新增的 `?? docs/map_task_asset_authoring_alignment_plan.md` | 保留该文件；仅增加本 A0 记录和 contract fixtures |
| navigation runtime | `feat/field-real-vehicle-integration` / `cf7de6c2fe80089e0f771c9f2c96cb6471d62f76` | **DIRTY — DO NOT TOUCH**：`src/agt_navigation/launch/navigation.launch.py` 已修改；Gazebo 文档 / 包为 untracked | 只读 schema / validator / publisher / Route loader 与 fixture；未运行其仓库测试、未写入该仓库 |
| navigation V4 | `main` / `8d3d7de833bad8b65aecf470bcf399c22023dc8c` | **DIRTY — DO NOT TOUCH**：migration audit / plan 文档为 untracked | 只读用户要求的 contract audit；未改动 |
| `agt_robot_description` | `experiment/greenhouse-task-continuity` / `b9635d1812080a0fb0f334e35f99801ec7a47c57` | CLEAN | 只读 Profile、schema；未改动 |

固定 mapping 仓库与上级目录未发现生效的 `AGENTS.md`。上表旧仓库工作区状态是本轮直接读取的状态；本轮结束再核对并记录是否保持不变。

## 已冻结的身份与 digest 字段

**不同 identity 与 digest 不互相替代。** 文件 digest 是对精确文件字节的 SHA-256；目录/包身份来自现有 ID / revision 元组；只有现有 Site summary 的 `map_hash` 是跨其声明资产的 aggregate 内容摘要。没有新增“Map Bundle hash”或第二种 Route hash。

| 资产 | Identity（不含内容 digest） | 内容完整性 / revision digest 来源 | 唯一 validator（现有） | 唯一 publisher owner 决策 |
| --- | --- | --- | --- | --- |
| PGO `mapping_source` | `package_kind=mapping_source` + `metadata.site/version` + backend/status | `manifest.yaml` 的 payload file list；`checksums.sha256` 中每个文件的字节 digest。现有 producer 排除 manifest，但现有通用 validator 将 manifest 计入实际文件集 | mapping `verify_artifact()`；其当前契约与 PGO producer 结果不一致，见 blockers | mapping `MapPackageExporter.export()`；现状是 IMPLEMENTED producer，validator coverage 不匹配 |
| Same-session frontend source reference | `artifact_kind=frontend_mapping_map_package` + backend / source reference | frontend manifest 覆盖 payload；checksum index 校验 payload 与 manifest。reference 明确不是 optimized PGO 或绝对真值 | mapping `verify_frontend_map_package()`（通用入口按 artifact kind 转发） | mapping `write_frontend_map_package()` |
| immutable deployed Map revision（Site 1.0） | `site.id` + `site.revision` | manifest 原始字节 `manifest_sha256`；每个 `assets.*` 文件和导航 PGM 均由 `hashes.yaml` 覆盖。Site `map_hash` = SHA256(`manifest_sha256 + 按路径排序的有效 path:digest 行`)。不哈 `manifest.yaml` 自身的 digest 字段、不包含任意未声明目录 | runtime `SiteValidator`（目录身份）→ `agt_runtime_contracts.validate_runtime_contracts()`（Site schema、path、安全、hash、Profile compatibility） | **RECOMMENDED target: mapping-owned Site 1.0 publication adapter**, 复用 reviewed publisher 规则；该 adapter 尚未实现。当前既有 writer 是 runtime `CommissioningService.save_review()`，仅作被复用的旧 publisher 实现，不新增并行 writer |
| READY Route revision | `route_id` + integer `revision` + frame | `route.csv_sha256` 覆盖 CSV 字节；binding 中 `route_manifest_sha256` 覆盖 Route manifest 原始字节。Route manifest 不写入自己的 digest。Task content hash 是另一身份，不能充当 Route hash | runtime `RouteTaskResolver` 执行 binding gate 并调用 `load_route_asset()` 验证 Route bytes / Map / Profile；这是一个验证链，不是第二套 validator | **RECOMMENDED target: mapping-owned READY Route publisher**, 复用现有字段 / reader；当前通用 authoring publisher NOT FOUND，runtime smoke / unit tests 会制造 fixtures，但不算正式 publisher |
| greenhouse structure annotation | `schema_version=1` + frozen output 内容；source 明确绑定 mapping source manifest/content hash | `*.manifest.json` 覆盖 annotation YAML 与 sidecars；source Map hash 是 mapping manifest bytes 的 hash 语义，不是 Site `map_hash` | mapping `validate_topology()` | mapping `save_topology(..., freeze=True)` / immutable freeze path |
| Geometry Evidence sidecar | `artifact_type=spatial_geometry_evidence_v1` + parent PGO / confidence source binding | sidecar `checksums.sha256` 覆盖 PCD 与 metadata；metadata 保存 parent manifest hash、parent checksum-index hash、confidence checksum-index hash。绝对 source paths + hashes 是当前绑定要求 | `load_geometry_evidence()` | `export_geometry_evidence()`；为独立 evidence sidecar，不是 Site localization layer publisher |
| Static localizability / ambiguity / anchor layer | **当前无已发布 identity** | 不定义新字段或 hash；应基于未来明确的 typed producer / validator | **NOT FOUND** | **NOT FOUND**；authoring/API 应对当前未注册类型返回 `UNSUPPORTED`，不能把 UNKNOWN 编成数值或 occupancy |

### Mapping PGO source 的当前 checksum 结论

当前 [MapPackageExporter](/home/yangxuan/ros2_ws/src/agt_mapping_framework/artifacts/agt_mapping_artifacts/agt_mapping_artifacts/map_package_exporter.py:25) 在生成 manifest 前写 checksum index，因此两者均不包含 manifest 自身 digest。此设计没有自引用。当前 [verify_artifact](/home/yangxuan/ros2_ws/src/agt_mapping_framework/artifacts/agt_mapping_artifacts/agt_mapping_artifacts/validation.py:77) 却把所有文件（仅排除 `checksums.sha256`）放进 expected coverage，包含 manifest。用临时最小 PGO source 运行现有 exporter 后调用现有 validator，实际拒绝 `Incomplete checksum coverage; unlisted=['manifest.yaml']`。A0 冻结结论：**manifest 自引用禁止；PGO producer / validator coverage 必须在 A1 统一后再把此包标成 READY**。这里保留生产和校验器各自行为，不在 A0 改码。

frontend source 的 writer 顺序不同：payload manifest 先写，然后 checksum index 纳入 manifest；manifest payload map 排除 manifest 自身。Site hashes 文件覆盖的是声明资产和 Nav PGM，summary 的 aggregate digest再显式纳入 manifest digest。Route 使用 CSV digest 与外部/Task binding manifest digest。以上 digest 不得互换或把文件 hash、Site aggregate hash、Task content hash改名。

### Map → Route 的精确绑定

冻结的逻辑引用为：

```text
MapRef = (site.id, site.revision, SiteSummary.map_hash)
RouteRef = (route_id, revision, sha256(route.yaml bytes), sha256(route.csv bytes))
Route.map_binding = (map_id, map_version_id, map_content_sha256)
Route.vehicle_binding.platform_profile_sha256 = selected validated profile digest
```

交接到现有 READY Route 字段时，`map_id == site.id`、`map_version_id == site.revision`；`map_content_sha256` 承载同一个 Site aggregate digest 的 64-hex 值，按 Route 既有 encoding 写作 `sha256:<hex>`。前缀只作现有字段表示，不重算 digest。Profile 绑定是 exact validated profile digest。任何一个 ID、revision、map digest 或 profile digest 不相等都拒绝。

mapping A0 fixture 先验证 Site 1.0 并用 canonical Site summary 得到 `map_hash`，再以该精确值加载 READY Route fixture，检查 id / revision / hash 的 tuple 相等；Route 单体字段和 CSV hash由当前 loader验证。这个 fixture 表达并检查 target binding 规则。**生产 resolver 尚未完成**：现有 `RouteTaskResolver` 假设 `maps/<id>/versions/<version>/manifest.yaml` 带 `state=READY` 与 `map_content_sha256`，Route 位于该版本目录；Site 1.0 使用 `sites/<id>/<revision>/manifest.yaml`，summary 使用 aggregate `map_hash`。所以当前 runtime 不会直接从 Site 包完成该 binding，A0 不能报告 production integration PASS。

Route ID/revision 与 Route file hashes 仍需进入调用方的不可变执行身份。既有 `.route.yaml` binding 会校验它引用的 Route manifest digest，但 V4 audit 已确认该 sidecar 不进入旧 Task canonical hash；它不能证明调用方预览的 Route 就是执行的 Route。消费 Site / Route 的 V4 adapter 后续必须按 audit 将 exact `task.route_ref` 纳入版本化 Task hash；mapping A0 不改 V4 Task schema / runtime。

## Authoring operation DTO 与结果语义（冻结为 PROPOSED）

以下是 A0 给未来 client / mock 的唯一推荐 envelope，**PROPOSED / NOT IMPLEMENTED**。当前 MapStudio `WorkflowSession`、`ExternalToolRunner` 是内部 API；不能当作此 DTO 的实现。A0 不创建 JSON Schema、ROS service、HTTP endpoint 或正式 C++ API。

```yaml
request:
  api_version: agt.authoring/v1
  request_id: <idempotency key>
  session_id: <draft session>
  expected_draft_revision: <integer>
  operation: <registered operation name>
  payload: <operation-specific fields>
  source_ref: <exact current asset ref, when needed>
  profile_ref: <validated profile identity + digest, when needed>

response:
  request_id: <echo>
  status: SUCCEEDED | INVALID | STALE | TAMPERED | UNSUPPORTED
  draft_revision: <integer, when accepted>
  job_id: <owned background job, when asynchronous>
  asset_refs: <exact identities returned after validation/publication>
  issues: [{code, object_id, field, reason}]
```

`operation` 初始 registry 只列 `inspect_asset`, `edit_geometry`, `edit_occupancy`, `generate_navigation_map`, `edit_structure`, `validate_structure`, `edit_route`, `import_trajectory`, `preview_route`, `validate_route`, `confirm_review`, `publish_map`, `publish_route`, `export_research_inputs`, `get_job`, `cancel_job`。registry 只冻结命名和语义；不宣称入口已存在，也不推导 ROS transport。

| 状态 | 冻结触发条件 | 与 UNKNOWN 的关系 |
| --- | --- | --- |
| `SUCCEEDED` | 操作完成且所有输出通过 authoritative validator；发布成功返回 exact ref | 缺 optional layer 本身可成功，但须标 absent / unknown |
| `INVALID` | schema / 数值 / frame / exact Map identity / Profile identity 不合法 | UNKNOWN 不是合法数值的 0，也不是 INVALID |
| `STALE` | request 的 draft revision 或 source digest 与已打开 session 当前值不同；要求刷新草稿 / 预览 | stale 旧预览不能发布 |
| `TAMPERED` | declared file / manifest / index digest 与实际 bytes 不匹配 | 必须返回具体资产和 digest issue |
| `UNSUPPORTED` | operation、typed layer、schema major 或所请求的 profile/controller capability 未注册 / 未实现 | unknown optional type不静默透传或当作空资产 |

`UNKNOWN` 是资产字段、观测值或覆盖状态；需要携带原因（例如未标注、无 GT、数据不足）。它不是响应状态，不能默认为 0 / false / free occupancy。验证器必须区分“可选且缺失”、“值为 UNKNOWN”、“格式或几何非法”及“正确类型但当前不支持”。

fixture 中 stale / unsupported request 是目标行为样例，现有 Facade 不存在，因此它们只能做静态 case coverage；不能记成运行时拒绝测试通过。

## Fixture 与已运行检查

| Fixture / 用例 | 既有 authority | 本轮验证方式 | 结果 / 边界 |
| --- | --- | --- | --- |
| valid Site 1.0 + exact READY Route | SiteValidator + Site summary；Route loader | mapping A0 pytest fixture 链接真实只读 validator / loader，检查映射 tuple | PASS：fixture表达且验证 exact digest / IDs；production resolver仍缺 |
| invalid / DRAFT Route | Route loader | Route status 设为 DRAFT | PASS：返回 `route_not_ready` |
| stale source / wrong map hash | Route loader | current Site summary 与过期 Route map hash比较 | PASS：Route loader 拒绝 map binding hash mismatch；Authoring status `STALE`/`INVALID` projection未实现 |
| wrong vehicle profile | Route loader | 用不同 expected Profile digest | PASS：返回 `route_vehicle_binding_mismatch` |
| tampered Site asset / Route CSV | SiteValidator / Route loader | 修改 fixture bytes，不更新 frozen hashes | PASS：Site `HASH_MISMATCH`，Route `route_csv_hash_mismatch` |
| unsupported operation / optional layer | 当前无 Facade / typed layer registry | matrix 要求 `UNSUPPORTED`；只验证静态 case 已登记 | **NOT EXECUTABLE**；A0 缺可执行规范断言，生产 Facade 的实现另属后续阶段 |
| UNKNOWN value | 当前 structure schema的 UNKNOWN 标签语义 / authoring DTO语义 | matrix检查 UNKNOWN fixture独立于 INVALID status | PASS for contract fixture only；通用 Authoring API未实现 |
| normal same-session frontend source | frontend writer + frontend validator | 实际 writer生成临时 source，通用 mapping入口验证 | PASS |
| current PGO exporter→generic validator | MapPackageExporter + verify_artifact | temp PGO source导出后运行 validator | **REPRODUCED FAIL**：manifest coverage mismatch；不修复（属于 A1） |

已新增 fixture 说明在 [A0 fixture case matrix](/home/yangxuan/ros2_ws/src/agt_mapping_framework/tests/contract_fixtures/a0/cases.yaml)、Site / READY Route 样例在同目录。pytest 只在 mapping repo 的临时目录生成数据，runtime / V4 / robot description 作为只读 import / schema 来源；使用 `PYTHONDONTWRITEBYTECODE=1`，不向历史 repo 写 pycache 或 pytest cache。

本轮执行：`PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider tests/contract_fixtures/a0/test_contract_fixtures.py -q` → **10 passed**。其中一个测试通过“预期抛出校验错误”记录当前 PGO exporter / validator coverage 缺口；10 passed 不表示该生产路径可发布。

## 十项 A0 验收结论

| # | 验收项 | 结论 |
| --- | --- | --- |
| 1 | 每个正式资产唯一推荐 authoritative validator | **PASS with scope**：按上文矩阵唯一指定；Route 是 resolver→loader 单验证链。静态 localizability / ambiguity / anchor layer 没有正式实现，因此不冒称资产 |
| 2 | 每个正式资产唯一推荐 publication owner | **PASS as owner decision**：目标 owner 唯一指定；Site mapping adapter / READY Route publisher尚未实现是后续实现限制 |
| 3 | Map / Route / optional layer identity无 digest混用 | **PASS as frozen rule**；Route 内部映射需要未来 adapter接入 |
| 4 | manifest自身不进入自引用 hash | **PASS as frozen construction rule**；PGO checksum indexing不一致已发现并标 blocker |
| 5 | exact Map→Route binding fixture可表达 / 验证 | **PARTIAL**：正例tuple相等已检查；错误map_id测试实际证明loader接受，尚缺ID/revision/manifest负例的完整规范拒绝验证。生产Site-to-Route不兼容另列后续限制 |
| 6 | stale source / wrong map / wrong profile / tampered hash负例拒绝 | **PARTIAL**：现有测试拒绝digest mismatch/profile sentinel mismatch/tampered bytes；source-session stale、draft revision、wrong ID/revision、unsupported尚缺可执行规范断言，真实Profile digest来源仍需闭合 |
| 7 | UNKNOWN 与非法值语义分离 | **PASS as frozen rule + fixture**；无 API 接收层运行验证 |
| 8 | V4只读消费、不修改runtime active state | **PASS as ownership contract**：Site publisher产出不可变 revision；仅V4 runtime决定active。跨仓库 runtime 测试未运行 |
| 9 | 未实现AuthoringFacade明确 PROPOSED | **PASS**：本文 DTO / operation均标记 PROPOSED / NOT IMPLEMENTED |
| 10 | 不修改旧发布资产 | **PASS**：fixture全部新建在 mapping tests；旧仓库、既有 Site / Route资产只读 |

因此整体保持 `PARTIAL`。没有据此启动 A1。

## A0 blocker 与后续动作边界

1. PGO `MapPackageExporter` 与 `verify_artifact` coverage 不一致。A0先冻结目标coverage/构建规则，A1再修producer/validator实现；本轮不修业务源码。
2. 补exact Map ID/revision/digest、Route manifest、Profile identity/digest及source-session/draft revision负例；现有loader缺口测试保留。Site→Route生产resolver适配是后续工作。
3. operation payload仍为占位；需统一名称、transport/serialization、字段类型/必填/unknown-key、版本/issue优先级以及UNKNOWN/INVALID/UNSUPPORTED的可执行规范样本。无需提前实现A2 Facade。
4. 当前结构/geometry的独立identity、实际Profile digest来源，以及新版source/block/query/evidence/intervention/HMI能力边界须补充小合同，继续复用现有hash与资产schema。

后续实现限制：Site mapping publisher、READY Route publisher、typed新layer producer、生产Facade和V4 consumption adapter均NOT IMPLEMENTED；不计为A0必须实现的业务代码。

下一推荐动作改为只在docs/contract fixtures中**补齐A0**，验收后再单独授权A1或适用研究子任务。**A1–A7未开始；不得因需求文档更新自动实施。**

## 本轮变更与验证边界

变更仅限本 acceptance记录与 `tests/contract_fixtures/a0/`；不触碰 LIO / PGO业务逻辑、BBS / GICP、MapStudio、Route tracker、Task executor、runtime activation、Robot Platform或历史仓库。pytest结果只证明本文件列出的fixtures和被调validator行为；不证明ROS build、UI、route发布产品、实车或runtime integration。

2026-10-06需求调整仅另更新总体plan、README入口并新增AGENTS/AI任务导航；未改变既有fixture bytes，也未重跑新增合同测试。上列10 passed是此前复核的结果，新需求仍为PROPOSED。详见[需求计划](../map_task_asset_authoring_alignment_plan.md)与[最小阅读集](../ai_task_entrypoint.md)。

随后用户指定默认FAST-LIVO2 LIO-only、不使用FAST-LIO2、关键帧分块不以PGO优化为前置；本次只更新需求/AI规则，实际backend默认与旧入口尚未迁移。PGO负例是旧producer缺口证据，不是要求默认必须运行PGO。新增同bag cold/global-loss配对协议为PROPOSED，既有centered-query smoke不能记为该协议已验证。
