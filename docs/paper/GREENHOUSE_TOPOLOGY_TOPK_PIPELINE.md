# 可复用的地图拓扑与 Top-K 实验流水线

本工具适用于新的温室/通道场景。路径、backend、frame 和时戳来自输入地图包，不硬编码当前温室行号。物理拓扑由人工确认；reference 是同 session frontend，不是绝对 ground truth。

本机本轮隔离安装环境（以下 ROS 命令先执行这段）：

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/experiments/greenhouse_topology_topk_20261003/colcon_install/setup.bash
export LD_LIBRARY_PATH="/home/yangxuan/ros2_ws/.agt_native/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

换机器或新 build 时替换隔离 install 和 native library 路径。标注 shell 入口可直接从源码运行；GLOBAL trace 需要支持 instrumentation 的 native binary。构建方法见工具 README，native patch 位于 `topk/native-instrumentation.patch`。

## 1. 打开新地图进行标注

直接从仓库运行，无需重建正式工作区：

```bash
cd /home/yangxuan/ros2_ws/src/agt_mapping_framework
./scripts/annotate_map_topology.sh \
  /path/to/new/map_package /path/to/new/annotation
```

多 backend 时以 Point-LIO 作为 canonical，只标一次：

```bash
./scripts/annotate_map_topology.sh \
  /path/to/point_lio/map_package /path/to/annotation/draft \
  --other-map-package /path/to/fast_livo2/map_package
```

也可使用构建后的 ROS 包入口：

```bash
ros2 run agt_greenhouse_annotation greenhouse_annotator \
  --map-package /path/to/new/map_package \
  --output /path/to/new/annotation/greenhouse_topology.yaml
```

GUI 中选择 Draw row centerline，依次点击顶点，右键/Enter 完成，输入真实物理行 ID、宽度、方向并确认；Draw headland polygon 类似。Manual scene marker 点击轨迹选择 scene 类型和 timestamp。Edit vertices 可拖动顶点；Edit details 可修改 ID、宽度及顶点；Delete、Undo/Redo、Save、Validate 位于侧栏。

未知行和未覆盖位置保持 UNKNOWN。自动 scene suggestions 只是供人工审阅的 CSV，不会创建物理行或替代人工 scene marker。

地图需要 map.pcd、patches、poses_timed.txt、metadata.yaml、manifest.yaml；manifest 校验、坐标 frame、backend/source hashes 绑定标注。显示云经 voxel 和点数上限裁剪，原始地图不修改。

## 2. 校验并冻结

人工检查行 ID、几何和 scene markers 后点击 GUI Freeze。CLI 等价流程：

```bash
ros2 run agt_greenhouse_annotation greenhouse_annotation_freeze \
  --topology /path/to/annotation/draft/greenhouse_topology.yaml \
  --output /path/to/annotation/frozen_v1/greenhouse_topology.yaml \
  --confirm-manual-review

ros2 run agt_greenhouse_annotation greenhouse_annotation_validate \
  /path/to/annotation/frozen_v1/greenhouse_topology.yaml \
  --require-frozen --verify-map-files
```

有其他 backend 时 freeze 加 `--other-map-package /path/to/other/map_package`，导出该 backend 的 scene correspondence 与 keyframe labels。Frozen 文件禁止修改；再编辑时新建 draft version，保存到独立目录。

输出 YAML、GeoJSON、keyframe_topology_labels.csv、annotation manifest、版本快照、验证报告。人工 scenes 会导出 `benchmark_scenes_<backend_id>.yaml`，可直接用于 benchmark；时间无法匹配或五帧窗口越界的 scene 会明确排除。

## 3. GLOBAL trace：新地图

权威 descriptor/BBS/GICP 位于 navigation。使用支持 trace 的隔离 native binary。新地图第一次需要构建 heldout assets，由现有 native builders 完成：

```bash
ros2 run agt_map_localization_benchmark agt_greenhouse_relocalization_benchmark \
  --map-package /path/to/new/map_package \
  --scenes /path/to/annotation/frozen_v1/benchmark_scenes_point_lio.yaml \
  --run-id new_scene_topk_v1 --output-root /path/to/experiments \
  --ros-install /path/to/isolated_install \
  --mode global --profile smoke --frames 1,3,5 \
  --skip-coarse-seed-local \
  --trace-candidates --candidate-top-k 10 --descriptor-prefilter 40 \
  --native-localizer-path /path/to/instrumented/candidate_bbs_gicp_localizer
```

所有人工声明 query 的 1/3/5 帧窗口从 target/descriptor 数据中排除。`--skip-coarse-seed-local` 省略 supplementary LOCAL 诊断，只运行原生 GLOBAL pipeline。原 benchmark 的默认行为保留。

默认 native K=2、wrapper/旧 benchmark K=4 未被改成 10。上述 K=10 是明确的实验参数。

## 4. GLOBAL trace：复用已冻结 assets

已有完整 benchmark 时，避免重复构建和大规模重跑：

```bash
ros2 run agt_map_localization_benchmark topk_trace_replay \
  --map-package /path/to/new/map_package \
  --scenes /path/to/scene_config_used_by_prior_benchmark.yaml \
  --benchmark-assets-root /path/to/completed_benchmark \
  --native-localizer-path /path/to/instrumented/candidate_bbs_gicp_localizer \
  --output /path/to/new_topk_trace_run \
  --backend-id point_lio --scene-ids MIDDLE_01,HEADLAND_01 --frames 1,3,5 \
  --candidate-top-k 10
```

入口核对 prior manifest、query exclusion、scene/map/asset hashes；每个 N 独立使用对应融合 query，重新调用 native retrieval。保存实际命令、raw native trace、带 reference/provenance 的 trace 和 run manifest，结束时确认输入字节未变化。输出目录必须全新。

## 5. 计算指标、导出论文表图

```bash
ros2 run agt_map_localization_benchmark topk_ambiguity_analysis \
  --traces /path/to/new_topk_trace_run/traces \
  --topology /path/to/annotation/frozen_v1/greenhouse_topology.yaml \
  --keyframe-labels /path/to/annotation/frozen_v1/keyframe_topology_labels.csv \
  --output /path/to/paper/topk
```

输入只读并校验 hashes。候选映射到 `(r,s,d,psi_rel)`；跨 backend 必须有显式 backend-to-canonical transform 和时间对应/残差报告。

输出要求的 8 张 CSV、7 张 PNG，以及逐 query metrics、逐 candidate coordinates、scene bootstrap 表和 manifest。Recall@1/3/5/10 同时区分 physical row 与 same-row |Δs|<1/2/5 m；UNKNOWN 不当作错行；HEADLAND 的 row-conditioned 指标通常 N/A，另报区域 recall。默认 entropy 使用等候选权重的 rank-frequency，softmax 必须显式指定 tau。

BBS/GICP/Acceptance 按实际观察分阶段。未运行和 timeout 删失；winner-only GICP 无法代表所有候选；真实 runtime acceptance 未记录时为 N/A。具体定义见 `EXPERIMENT_VARIABLE_DEFINITION.md`。

## 本轮版本与真实 smoke

实现位于 `feature/greenhouse-topology-topk-v1`。native 在隔离 worktree/build/install；正式 navigation HEAD 和 install 未覆盖。native source patch 随本目录 `topk/native-instrumentation.patch` 保存。

本轮真实地图 GUI load/save、完整地图校验、baseline/OFF/ON parity 已完成。当前真实标注是 empty draft，100% UNKNOWN；没有伪造物理行。用户可用上述工具在下次新场景手工标注。本轮 trace/表图可用 `--allow-draft` 生成 instrumentation smoke，物理行 Recall、错行、沿行歧义和行 entropy 保留 N/A。

详细 smoke、测试数量和限制将记录在 `topk/RUN_REPORT.md`，原始实验根目录：
`/home/yangxuan/ros2_ws/experiments/greenhouse_topology_topk_20261003`。
