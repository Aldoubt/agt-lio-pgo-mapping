# 温室覆盖路线 authoring

`agt_coverage_authoring` 是建图完成后的独立离线功能包。它消费已审查的二维地图、人工确认的行道/作业区以及 Robot Profile，生成可检查的覆盖路线、净空与质量报告。MapStudio 是它的操作入口；定位、在线避障、跟踪、TF、底盘控制和 active-map 选择仍属于导航/平台仓库。

本次功能覆盖逐行遍历与按幅宽覆盖两种模式，前进优先、允许必要的倒车。首版以离线生成、审查、保存及既有资产接口兼容为终点。软件生成成功、几何可行、动态约束已验证、READY 资产兼容和实车执行是不同状态。

## 输入与操作

先完成现有 PCD→栅格→人工审查流程，保留原点、分辨率和地图朝向。不要将精修后的 PCD 与旧 patches 派生的栅格混用。缺少车辆高度检查的地图也不能由二维路径验证补出高度通行性结论。

1. 在 MapStudio 中生成行道候选，修正边界、行道端点、宽度和地头。自动提取先针对近似平行的直行结构；外围行道、缺行、曲行与分叉需要人工确认。
2. 确认结构使用 `map` 米制坐标。`map_reference`、`camera_init` 等 frame 不能直接改名当作 `map`；需要已有、可追溯的变换。
3. 选择经过审查的地图 YAML、Robot Profile、确认结构、覆盖模式和起终位姿。面积模式必须填写实际作业幅宽。
4. 生成净空分析或覆盖草稿，检查未覆盖对象、碰撞、运动学和动态约束状态。修改输入后重新生成和审查。
5. 保存完整草稿；仅在权威资产与 Profile 校验、当前运行端能力和发布条件全部通过时生成 READY Route。

Studio 的结构导出支持显式确认平面 `reference → map` 变换（dx、dy、yaw）；非 map 源即使使用零变换也需明确确认。它只变换导出副本，保留原始标注及摘要，并记录变换与地图摘要。需要利用已审查的 transition region 作为地头时，在导出选项中明确选择；它们仅参与连接和端点车体包络，不自动加入作业面积。

CLI 的基本入口：

```bash
agt_coverage_authoring analyze --request /path/to/request.yaml --output /path/to/new-analysis
agt_coverage_authoring plan --request /path/to/request.yaml --output /path/to/new-plan
agt_coverage_authoring --help
```

分析、规划、验证和草稿导出必须使用新的结果目录；READY 导出可使用已有 companion store，但不能覆盖已有 Route revision。CLI 与 Studio 使用同一算法和验证入口；可取消的后台任务不改变采集进程或运行中地图。进程失败、输入过期或取消不产生可发布的成功结果。

`analyze` 写出 `analysis.json`、`clearance.npz`、`inputs.json` 和 `report.json`；`plan` 额外写出 `plan.json`。距离层记录米制净空、有效性 mask、分辨率和原点。`plan.json` 保留完整点列、分段方向、作业身份、未覆盖目标及输入摘要。命令退出成功表示操作完成，仍需检查报告中的 `complete` 和验证状态。

复查与草稿导出使用原请求和保存的计划：

```bash
agt_coverage_authoring validate --request /path/to/request.yaml \
  --plan /path/to/new-plan/plan.json --output /path/to/new-validation
agt_coverage_authoring export --format draft --request /path/to/request.yaml \
  --plan /path/to/new-plan/plan.json --output /path/to/new-draft
```

草稿导出写出既有列格式的 `route.csv`、`status: DRAFT` 的 `route.yaml` 及报告。它可以保存完整倒车候选，也可以保存明确不完整的任务，现有 READY reader 会拒绝它。

请求是操作参数，不是第二套 Map/Route/Profile schema。路径相对请求文件解析。最小形状如下：

```yaml
map_yaml: ./confirmed/map.yaml
robot_profile: ./profiles/greenhouse_ackermann.yaml
structure: ./confirmed/coverage_structure.geojson
mode: rows                 # rows 或 area
backend: ros               # reference 必须显式选择
backend_options:
  ros_domain_id: 91         # 独立离线 domain；不能等于调用进程的运行时 domain
start: [2.0, 3.0, 0.0]     # 后轴中心 x/y 与车身 yaw，单位 m/rad
end: [2.0, 3.0, 0.0]       # 缺省返回 start
allow_reverse: true
operation_width_m: 0.5     # area 必填；不是车宽
clearance_margin_m: 0.0    # 不隐式重复增加 Profile 的安全边界
```

结构输入使用 GeoJSON FeatureCollection，显式 `frame_id: map`、`review.status: CONFIRMED`，每个启用对象需要 `properties.id`、`feature_type` 和 `review_status: CONFIRMED`。`field_boundary`、`keepout`、`headland`、`work_area` 为面，`aisle` 为中心线，可附带 corridor 面和允许通行方向。Studio 新的行道候选有结构化身份和端点链接；规划不得依赖自由文本备注恢复语义。确认前候选保持草稿，低置信度不自动变为可通行。

## 两种任务及覆盖口径

| 模式 | 必须完成的对象 | 报告口径 |
| --- | --- | --- |
| `rows` | 选定的完整行道 | 完成行道数/目标行道数、缺失行道及原因、重复经过和连接距离 |
| `area` | 选定的作业区 | 实际幅宽扫掠的覆盖面积、漏作面积、重叠面积和覆盖比例 |

面积模式默认工具居中、左右对称。地头用于换行，未明确选为作业区时不进入必作业面积；两侧作物喷洒模型、工具偏置和非对称幅宽不在首版范围。曲线与连接段的几何长度不等于作业覆盖。

未知、障碍、无法连接和搜索未完成的目标仍保留在任务分母中。部分路线保留可检查结果及原因，不能通过删除困难行道提高完成率。搜索预算耗尽表示未找到候选，不证明物理上绝对无路。

## 距离场、车体与阿克曼验证

二维 EDT 用于净空热图、狭窄区域筛选和路线比较；未知单元保留独立有效性标记。完整车体和挂载轮廓的带朝向扫掠仍是碰撞依据。首版不引入三维 ESDF，也不通过距离场把未知空间改成 free。

内部轨迹位置以后轴中心为准，yaw 表示车身朝向。`direction` 显式区分前进与倒车，`v_ref` 是非负速度幅值；换向点保留停车事件，不能通过 yaw 翻转伪造转向。验证检查侧移、原地旋转、曲率、碰撞和换向语义。动态限速、转角变化率及时间信息不足时，结果保留 `UNKNOWN`。

v2 的旧 `mk_mini` Profile 仍有差速声明；不得作为阿克曼真值。`greenhouse_ackermann` 可用于未标定的离线预览，但外形、后轴到 `base_link` 偏移、转弯半径、限速及转向速率需要统一 Profile 中的真实依据。原厂不同版本尺寸/半径有差异，网页规格不等于当前实机标定。

## 后端与依赖

核心不导入 Qt 或 ROS。它使用 NumPy、SciPy、Shapely、Pillow 和 PyYAML；ROS backend 的导入和连接在显式选择该后端后进行。

在仓库根目录安装并运行无需 ROS 的合成示例：

```bash
python3 -m venv /path/to/coverage-venv
source /path/to/coverage-venv/bin/activate
python3 -m pip install -e 'planning/agt_coverage_authoring[reference,test]'
agt_coverage_authoring plan --request planning/agt_coverage_authoring/examples/rows.yaml \
  --output /tmp/coverage-rows
agt_coverage_authoring plan --request planning/agt_coverage_authoring/examples/area.yaml \
  --output /tmp/coverage-area
python3 -m pytest -q planning/agt_coverage_authoring/test
```

示例为合成地图，明确选用 `reference` 后端和未标定 Profile。重复运行时选择新的输出目录；程序不会覆盖原结果。

ROS 部署使用 Humble 对应的 Python 与已构建的锁定覆盖依赖工作区，再构建本包。本包要求 Shapely 2；Humble 系统仓库可能提供 1.x，因此需在能访问系统 ROS 包的 Python 环境中安装核心依赖，不能只依赖 rosdep 的版本：

```bash
source /opt/ros/humble/setup.bash
source /path/to/pinned_coverage_ws/install/setup.bash
python3 -m venv --system-site-packages /path/to/coverage-ros-venv
source /path/to/coverage-ros-venv/bin/activate
python3 -m pip install -e planning/agt_coverage_authoring
colcon build --packages-select agt_coverage_authoring
source install/setup.bash
```

| 后端 | 用途 | 边界 |
| --- | --- | --- |
| `ros` | OpenNav/Fields2Cover 生成作业带，Smac Hybrid-A* 搜索连接 | 当前要求零 yaw 栅格原点；在独立离线 ROS domain 中仅运行地图、覆盖/规划服务及必要 lifecycle |
| `reference` | 无 ROS 的离线算法、数据和界面验证 | `rsplan==1.0.10` 提供有限解析连接候选，Shapely 处理参考作业带；不是障碍图搜索，不冒充 Smac 或 Fields2Cover 验收 |

后端缺失时显式失败，不自动从 ROS 切换到参考实现。[覆盖依赖锁定文件](../planning/agt_coverage_authoring/config/coverage_dependencies.repos) 沿用 v2 的五项完整提交：OpenNav `humble-v2` 为 `f413d0da7b4e52249b9abdb9d1fec7cef0238449`，Fields2Cover `v2.0.0` 为 `3613525c241538fa9fd9df3e1209ae8184627958`。这些是要求构建的锁定目标；客户端尚未证明运行中的安装产物来自对应提交。部署验收需要单独记录实际构建来源，不将报告中的锁定目标或 pip 上不同版本安装成功当作锁定链已验证。

ROS 模式默认自动启动并回收本次拥有的离线服务器进程组。请求中的 `backend_options.ros_domain_id` 必须明确且不同于调用进程的运行时 domain；默认 namespace 为 `/agt_coverage_offline`。同名服务器已存在时拒绝重复启动。只有显式配置 `auto_launch: false` 才连接已启动的离线服务器，并继续检查其规划参数。手动服务必须使用本包 `offline_backend.launch.py` 的同等配置和同一派生地图，不能指向机器人正在使用的全局 costmap。

核心与 `reference` 模式保留非零地图 origin yaw；ROS 模式为避免 Smac 轴对齐 costmap 误解释而返回 `UNSUPPORTED_MAP_YAW`，不会静默丢弃朝向。旋转地图应先通过独立、可审查的重栅格化流程产生新版本，再用于该后端。

面积模式先解析作业方向：请求的 `heading_rad` 优先，其次使用已确认行道方向，无行道时采用作业区最小旋转包围矩形的长轴。报告记录最终角度及来源，避免把旋转温室默认当作沿地图 X 轴排列。

行道遍历直接使用确认的中心线身份，包括单行任务。区域模式将作业带语义与连接语义分离；零长度或错配组件必须拒绝。换行先尝试前进，再考虑倒车，并比较相邻、隔行等候选顺序。连接失败可以改变访问顺序，不修改已确认的作业几何来掩盖失败。

## Site / Profile / Route 交接

草稿完整保留 F/R、SWATH/CONNECTION、行道身份和停车事件。可视化 Path 只是几何投影，不是完整任务资产。地图、结构、Profile 或参数的实际字节变化都会使已审查结果过期。

正式发布沿用 Site 1.0 与既有 READY Route 的 CSV/manifest。发布适配器调用安装的 V4 权威 Site、Profile、Route 组件；缺少它们时拒绝发布，不以本包的局部检查或测试替身替代权威 validator。Route 使用独立 companion store，不覆盖 Site 或既有 revision。

READY 发布配置需要在规划前通过请求的 `publication` 路径绑定；同时提供 `site_root` 和精确 `map_binding`。发布配置包含 `route_id`、`revision`、`site_schema`、`profile_snapshot`、`profile_digest`、`profile_snapshot_sha256`、`site_compatibility_key` 和 `route_reference_point: base_link`。其中 Profile 摘要和 compatibility key 必须来自既有权威 owner，不使用示例伪造值。配置文件不预先确认尚未生成的候选：检查本次计划之后，在导出时显式传入 `--confirm-review`；配置中的 `review_confirmed` 不能代替该操作。

```bash
agt_coverage_authoring export --format ready --request /path/to/request.yaml \
  --plan /path/to/new-plan/plan.json --publication /path/to/publication.yaml \
  --confirm-review --output /path/to/route-store
```

READY 写入 companion store 中新的 `routes/<route_id>/<revision>/`，不覆盖已有 revision，也不激活资产。修改发布配置后需要重新规划和审查；导出时的 `--confirm-review` 只确认当前输入绑定下的当前候选，不能把新配置强加给旧计划。

Profile 的两种摘要不能混用：

- 输入 YAML 的文件 SHA 用于发现本次 authoring 输入变化。
- 当前 V4 的 Profile content-closure digest 来自受验证 snapshot，涵盖其声明的完整配置闭包；READY Route 绑定该权威 digest。还需证明 snapshot 中的原始 Profile YAML 与本次规划输入完全一致。

当前 V4 Route reader 仅接受完整前向、统一速度的路线。需要倒车或变速的候选仍可审阅和保存，但发布报告保持不兼容；不得删去倒车连接后把剩余路线称为全覆盖。后轴轨迹向 `base_link` 的输出转换也必须明确约定，不可仅改变字段名。

现有 Route 合同还没有规定偏移车体参考点的曲率语义，因此当前适配器对非零 `rear_axle_to_base` 返回 `REFERENCE_POINT_UNSUPPORTED`。典型 MK-mini 几何中心与后轴相隔约半轴距：其完整规划草稿可用，但仅声明输出点为 `base_link` 仍不足以获得 READY。需要先在唯一共享合同中确认参考点和曲率语义，不能在本包旁设一个兼容格式。

v3 的现有 READY 查看器通过 V4 resolver 读取资产，因此可用于离线审阅兼容结果。本文不声明 v3 已有可执行的 MK-mini 倒车跟踪链。实车换挡、停车确认、跟踪与取消传播属于下一阶段导航能力验收。

## 验证方法与当前边界

自动化检查应覆盖单行/多行、窄口、未知区、地头不足、倒车换向、侧移拒绝、扫掠碰撞、覆盖分母、陈旧输入、取消及 immutable 导出。保存测试命令和实际结果，分别报告纯 Python、reference backend、真实 ROS backend、原生 GUI 和物理车辆。

在没有 ROS 2、锁定 OpenNav/Fields2Cover、Qt/PCL 构建环境或真实温室地图的环境中，对应集成应记为 `NOT_RUN` / `BLOCKED`。参考后端或小型合成地图通过，不能写成真实 ROS、GUI 或温室实车验收通过。

产品配置中 `route_authoring: manual` 仅登记手动入口；默认仍为 `disabled`。`--capabilities` 可以列出该能力，但不会启动它，也不会自动发布 READY Route。现有 A0 与其它产品阶段的未完成项保持原状态。

### 本次验证记录（2026-10-26）

- Python 功能包与产品登记回归：138 passed。包含真实 reference 后端、输入/取消/失效、双模式覆盖及调用未改动 V4 权威源码的 Site/Profile/Route 往返测试。
- 原生 Qt 回归：16 passed。其中 ResearchAnnotationModel 7 项，结构导出 8 项，取消清理 1 项；涵盖行道身份/端点链接、显式坐标变换、地头选择及超过旧 3 秒上限的清理。
- 独立 Qt 对话框离屏端到端检查：PASS。实际调用 reference CLI，确认完整路线、地图/净空/轨迹预览，并验证输入变化后清空预览和报告、禁用草稿导出。此检查直接编译对话框及其依赖，不等同于完整 MapStudio 构建。
- 两种合成温室示例均生成完整的 558 点路线：2/2 行道、12.8 m² 目标区域全部覆盖；未标定 Profile 的动态约束保持 UNKNOWN。这些数字不属于真实温室或实车验收。
- ROS OpenNav/Smac 原生联调、完整 MapStudio colcon 构建、真实温室地图及实车执行：NOT_RUN。

Python 验证命令（`AGT_COVERAGE_AUTHORITY_ROOT` 指向测试用的未改动 V4 权威模块及 schema；缺少它时权威集成用例明确 skip）：

```bash
PYTHONPATH=planning/agt_coverage_authoring:bringup/agt_mapping_bringup:artifacts/agt_mapping_artifacts:core/agt_mapping_core \
AGT_COVERAGE_AUTHORITY_ROOT=/path/to/pinned-v4-authority \
python3 -m pytest -q -p no:cacheprovider planning/agt_coverage_authoring/test \
  bringup/agt_mapping_bringup/test/test_product_pipeline.py
```

在具备完整 ROS/Qt/PCL 构建环境时，原生单元测试通过 `agt_map_studio` 的 CMake 测试目标运行；另可显式运行独立对话框 smoke（需要 PATH 中已安装带 reference 依赖的 CLI，输出目录必须尚不存在）：

```bash
QT_QPA_PLATFORM=offscreen ./build/agt_map_studio/coverage_authoring_dialog_smoke \
  planning/agt_coverage_authoring/examples/rows.yaml \
  /tmp/coverage-dialog-new-result /tmp/coverage-dialog-preview.png
```
