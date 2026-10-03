# 下一次番茄温室数据采集规范

本文件定义下一轮现场采集和离线实验的冻结规则。它是待执行的实验协议，不代表已经采集了新 session，也不代表当前 greenhouse smoke 获得了真实 physical row 标注。现有 `/home/yangxuan/rosbags/green-house` 仅用于工具链 smoke；上一轮 problem-existence 原始数据、报告和未提交文件应完整保留。

协议的直接目标是分别测量跨行混淆、同一行中的纵向混淆，以及多帧对 GLOBAL retrieval 和后续 BBS/GICP 的影响。算法权威实现继续位于 navigation repository；现场和分析过程不修改 Polar Context ranking、3D-BBS、small_gicp 或其 acceptance 规则。

## 1. Session 设计和现场记录

| Session | 采集条件 | 主要用途 | 必须记录 |
|---|---|---|---|
| A | 当前或低植被阶段；选择一个完整 session 作为建图参考 | 建图重复性、同 session retrieval 参考、physical topology 初次确认 | 日期、温室编号、机器人、传感器序列号、植被照片、行布局、地图 session ID |
| B | 另一日期，优先保留 A 的行布局和机器人配置 | 跨日期 query；避免把 A 的同一段扫描当作独立新 session | 日期间隔、光照、植被和设备变化、移动物体、reference-map ID |
| C | 明显不同生长期；如确有跨季节采集则单独标注季节 | 跨生长期 query；验证场景效应是否仍存在 | 生长期说明、植被高度/密度的观察记录、布局和传感器配置变化 |

三个 session 是最低建议设计，日期与生长期在采集前填写；不能把同一 bag 的三次回放命名成三个现场 session。若 B/C 的 row 布局改变，保存 row 的人工对应表和变更原因；不能只沿用坐标或名称。只有实际不同季节采集的数据才能支持跨季节论述。

每个 session 尽量使用同一套经人工确认的 physical row 身份，在现场挂可识别的 `R01/R02/...` 标签并拍摄布局草图。Row ID 表示真实物理行，不表示 LIO trajectory pass、序数分组或某个 descriptor patch。给至少两个相距较远的 headland 设 `H01/H02/...`。无法确认的位置记录 `UNKNOWN`；发生布局变更时新建 topology 版本。

每个 session 至少完成：

1. 每条可通行 physical row 双向各一次；记录行进方向、入口/出口和沿行里程标记。
2. 至少三次经过同一 headland，并从不同相邻行进入；保持经过路线和转向记录。
3. 在随机预选的 row-middle 位置停止、重新启动定位；在 headland 也完成停止/重新启动。
4. 动态干扰与正常试验分开记录。记录人、工具、移动设备进入视野的起止 bag 时间，不因干扰导致失败而删除该 trial。
5. 记录 sensor topic、message type、frame、时间来源、IMU/LiDAR extrinsic、软件版本和 sensor 参数；每次配置变化生成新 profile ID。

SESSION A 同 session 结果与 A-map/B-query、A-map/C-query 结果分表统计。跨 session query 的参考位姿需要独立建立；同 session frontend 参考不能直接作为跨日期 absolute GT。

## 2. 原始数据和时间契约

原始录制必须包含 MID360 `livox_ros_driver2/CustomMsg` 和原始 `sensor_msgs/Imu`。同时保存 wheel odom；若 wheel odom 缺失，明确标为 `NOT_RECORDED`。具备条件时保存 RTK、测量 control points 或全站仪参考，并保留其质量状态、时间同步和坐标转换记录。温室内 RTK 质量差的片段不能自动作为绝对真值。

当前 greenhouse reference 使用 `/agt/sensors/lidar/custom` 和 `/agt/sensors/imu/data`。下一次录制前应通过实际 topic/type 检查确认；不能仅复制 topic 名后假定传感器数据有效。录制原始数据和算法输出可同时进行，但原始数据是后续重新建图、adapter 审计和时间对应的基础。

Session manifest 记录：

| 字段组 | 必须保存的内容 |
|---|---|
| 时间 | UTC 时间、timezone、ROS header stamp 来源、bag receive timestamp、控制器/外部参考同步方式、测得的 offset/漂移 |
| LiDAR | topic/type/frame、CustomMsg timebase、逐点 offset_time 单位、message count、异常时间区间 |
| IMU | topic/type/frame、频率、gyro/acceleration 单位、extrinsic、message count、异常时间区间 |
| 辅助记录 | wheel odom topic/type、RTK/control-point 来源和质量、照片/人工事件日志 |
| 完整性 | rosbag metadata SHA256、bag storage 文件 SHA256、录制开始/结束、磁盘/丢包告警 |

停止机器人或重启 localization 时持续录制原始 bag，并写事件；不要为了获得一个成功案例而无记录地重启整份录制。若真实 power-cycle 必須切分 bag，用 session ID、segment ID 和事件表保留关系。原始 bag 只读，重放、点云派生和实验结果写到独立目录。

## 3. Physical locations 和重复 trial

ROW_MIDDLE 至少选择 5 个不同 physical locations，HEADLAND 至少选择 5 个不同 physical locations。Row-middle 尽量分散到多个真实行和纵向站点；headland 的五个位置可以位于多个 headland polygon 的不同站点。位置应在查看算法结果前选定并冻结，不能按成功/失败事后挑选。

给每个位置建立 `location_id`，保存现场照片、人工 row/headland ID、近似 longitudinal station、站点确定方法和不确定性。HEADLAND 没有 physical row 时写 `physical_row_id: null` 和 `row_status: NOT_APPLICABLE`；不要强制归最近行。人工 row 身份可以确认，而 longitudinal station 和 pose 仍可能只是近似参考。

| Trial 因子 | 冻结值/规则 |
|---|---|
| Scene `S` | `ROW_ENTRY`, `ROW_MIDDLE`, `ROW_END`, `HEADLAND`；主比较为 ROW_MIDDLE vs HEADLAND |
| 实机初始朝向 | 相对于行切线或预先定义的 headland 朝向为 `0°`, `+20°`, `−20°`；现场安全且可操作时追加 `+90°` |
| 重复次数 | 每个 physical location × 可执行 yaw 至少 3 次独立 stop/restart；保存 trial order |
| Query window `N` | 对同一个已冻结 trial/query anchor 派生 1、3、5 帧；按冻结的尾随/锚定规则积累并重新运行 descriptor、BBS、GICP |
| Retrieval `K` | 保存完整 prefilter ranking，离线算 1、3、5、10 的 descriptor 指标；BBS/GICP 的 K 效应用显式各 K 的独立运行 |
| Backend `B` | Point-LIO 与 FAST-LIVO2 LIO-only；先在新 session 完成 replay repeatability，再选择 canonical reference |

仅使用三个基本 yaw 时，最低现场 restart 数为 `2 classes × 5 locations × 3 yaw × 3 repeats = 90`。这产生 90 个 restart trial；对每个 trial 派生三个 N 产生 270 个 query-window 条件，不能宣称 270 个独立现场试验。安全允许的 +90° 要么在采集前纳入完整设计，要么作为有原因记录的额外条件；不可把未执行的 +90° 记为失败。

ROW_ENTRY 和 ROW_END 作为次要比较也建议各预选至少 5 个位置。若现场无法覆盖，提前记录缺少的因子组合，不补造结果。实际试验顺序随机化或使用固定随机种子；相邻重复 trial 的电量、光照、动态干扰和等待时间仍应记录。

不同 N 的 query 必须包含同一 query anchor，使用同一累计方法、motion compensation 和 query frame 契约。多帧内容不能跨越事后人工修正或不明时间重置。建图地图/descriptor 数据库排除规定的 query interval；同 session 派生窗口之间的重叠及排除范围写入 manifest，禁止 query 对自身 patch 的泄漏。

## 4. Trial manifest 和事件日志

以下是待填写模板，`null` 表示尚未测得或不适用；不应把这些示例当作已发生的试验：

```yaml
schema_version: 1
session_id: SESSION_A_REPLACE_DATE
trial_id: A_MIDDLE_LOC01_YAWP20_REP01
location_id: MIDDLE_LOC01
scene_state: ROW_MIDDLE
physical_row_id: R01
physical_row_evidence: manual_sign_and_photo
approx_longitudinal_station_m: null
station_method: null
station_uncertainty_m: null
headland_id: null
startup_bag_timestamp: null
startup_wall_time_utc: null
query_anchor_bag_timestamp: null
restart_type: localization_process_restart
initial_yaw_offset_deg: 20
query_frame_windows: [1, 3, 5]
manual_intervention: false
manual_intervention_events: []
first_accepted_bag_timestamp: null
first_reference_correct_bag_timestamp: null
recovery_time_s: null
timeout_s: null
outcome: PENDING
censor_reason: null
dynamic_interference_interval_ids: []
raw_bag_id: null
reference_map_id: null
topology_sha256: null
query_selection_sha256: null
algorithm_parameters_sha256: null
```

真实 trial 另存 `requested_action`、停止开始/结束时间、机器人是否完全静止、算法启动命令、二进制/动态库 SHA256、退出码、运行日志和每个 query/trace SHA256。记录 accepted 的时间和 reference-correct 的时间，以区分错误接受与实际恢复。

Recovery 计时起点是冻结的 startup 事件。终点定义为第一个同时满足 acceptance 和参考正确性条件的定位输出；可以预先要求持续一定秒数/连续若干输出以防闪现成功。持续时间/数量在采集前冻结。超时写 `TIMEOUT` 和右删失时长；人工接管写 `MANUAL_INTERVENTION` 和接管时间；不能把未恢复记为零秒或从成功率分母删除。

事件 CSV 最少含 `session_id, trial_id, event_id, event_type, bag_start_timestamp, bag_end_timestamp, wall_time_utc, actor, notes`。单次人工介入事件、动态干扰区间和安全取消都保留。安全取消、录制故障、算法超时是不同原因，汇总时分别报告。

## 5. 建图、标注、审核和冻结

1. 新 bag 先审计 topic/frame/time/extrinsic，再使用固定 profile 独立重放。记录 backend source commit、实际 executable 和动态库 hashes；不以源码 HEAD 推断 install 来源。
2. 为 Point-LIO 与 FAST-LIVO2 LIO-only 各建立重复回放记录。使用冻结的 repeatability 分类和原始 XY/yaw/cloud 指标选择 reference；repeatable 表示重放一致性，不保证物理轨迹正确。
3. 选择明确的 canonical backend/session map，启动 `greenhouse_annotator`。人工画每条 physical row centerline、名义宽度、方向和 headland polygon；记录人工确认依据和置信度。
4. 自动生成 `keyframe_topology_labels.csv` 与 scene candidates。检查沿行 `s`、有符号 `d`、relative heading、row corridor 重叠和 UNKNOWN 覆盖。Headland/row 重叠须按工具定义人工复核；未知区不能自动强制 row assignment。
5. 人工审核 scene markers，关联现场 `location_id` 和真实 row/headland 事件。最大 row assignment distance、timestamp tolerance 和区划优先级写进 topology/protocol。
6. 运行 annotation validator，保存 source manifest hash、coordinate frame、row/scene ID 唯一性、polygon validity、coverage 和 UNKNOWN percentage 的结果。
7. 冻结 YAML、GeoJSON、labels、scene selection、validator JSON 和 provenance manifest。旧版本保留，新版本用新目录/版本 ID；benchmark 只读 frozen annotation。

Row-relative 状态 `(r,s,d,psi)` 来源是 `MANUAL_TOPOLOGY + SAME_SESSION_FRONTEND_REFERENCE`。Polyline 点击顺序固定 `s=0`、切线方向和 `d` 正负方向，反向行驶不会自动反转 centerline。方向字段描述允许行驶方向，不改计算坐标。

第二个 backend 通过同 bag timestamp 建立 `backend_scene_correspondence.csv`，保留 keyframe ID、实际 stamp 与 `dt`。同名 `camera_init` 不表示坐标相同。需要把 candidate pose 变到 canonical 坐标时，必须保存独立核验的 transform 和残差；只有 timestamp scene 对应时，不得直接拿另一 backend 的 XY 套 canonical topology。跨 session 同理。

## 6. 冻结参数、依赖和输出

每次实验创建不可混用的 run manifest，至少锁定：

- 原始 bag、metadata、map package manifest/地图/poses/patches、descriptor/BBS assets、topology/labels/query selection 的 SHA256。
- mapping/navigation Git commit 与 dirty diff 快照，backend commit，annotation tool commit，native executable 与依赖动态库 SHA256。
- BBS、GICP、descriptor 全部参数，map/scan/assets voxel resolution，K/prefilter，共享及逐 candidate timeout、threads/CPU affinity、ROS domain、query frame 与 extrinsic。
- 正确性 thresholds、longitudinal thresholds `[1,2,5] m`、entropy 方法、score epsilon、bootstrap seed/重复次数和独立统计单位。

本机审计时 native `candidate_top_k=2`, `descriptor_prefilter=40`；现有 mapping benchmark 配置为 `candidate_top_k=4`, `descriptor_prefilter=40`。论文 smoke/仪表化模式显式请求 K=10，不能因此改运行时默认。现场实验以实际传入配置和 manifest 为准；默认值不代替冻结记录。

推荐 run 结构：

```text
dataset/SESSION_A/
  raw_bag/                         # 原始录制，只读
  session_manifest.yaml
  trial_manifest.yaml
  events.csv
  photos_and_control_points/
  mapping/<backend>/<run_id>/
  annotation/<version_and_hash>/
  queries/<selection_id>/
  global/<backend>/<N>/<K>/<run_id>/ # query、trace、stdout、stderr、配置与hash
  analysis/<analysis_id>/           # tables、figures、metrics/provenance
```

Paper 输出需要 retrieval、wrong-row、longitudinal ambiguity、entropy、margin、BBS/GICP stage、multi-frame 和 cross-backend 八张表，以及 topology、两类 Top-K 展布、Recall@K、entropy、longitudinal、failure breakdown 七类图。发布任何图表时附带 analysis manifest，保留其输入 hashes、有效分母和缺失原因。仅有同 session pose 的结果不能写 absolute localization accuracy。

## 7. 统计和结束检查

每个 query 是一条原始分析记录，Top-K candidates 不是 K 个独立样本。对于同 physical location 的 yaw/restart/N 相关重复，主 scene 效应以 location/scene 为 cluster 做 bootstrap，并在 session 内分层；多帧使用配对 query anchors。只有三份现场 session 时不能靠大量 candidates 或重复窗口获得跨 session 的窄 CI。

HEADLAND 的 physical-row Recall/WrongRow/Δs 在 query 没有 row 时为 N/A；使用独立 Headland-ID Recall 和 XY-region Recall。Candidate row entropy 仍可作为 candidate 分布指标，但必须同时给 row-labeled candidate coverage，避免把所有 candidate 落在 UNKNOWN/headland 的情况解释为低跨行歧义。

离场前检查原始 topics 和 message counts、trial/events 完整性、最低位置/重复数、安全取消和动态区间、照片/control-point 对应、磁盘和数据备份。分析前验证所有 manifest/hash；若缺 independent ground truth，就保持 manual topology + same-session reference 的证据等级。任何参数或标注修改生成新 run/version，旧 trial 和失败案例继续保留。

指标的完整公式、阶段删失规则及论文解释见 [EXPERIMENT_VARIABLE_DEFINITION.md](EXPERIMENT_VARIABLE_DEFINITION.md)。工具链当前实施状态见 [GREENHOUSE_TOPOLOGY_TOPK_PIPELINE.md](GREENHOUSE_TOPOLOGY_TOPK_PIPELINE.md)。
