# 编译与测试流程

本文适用于当前工作区布局：

```text
/home/yangxuan/ros2_ws/
├── src/agt_mapping_framework/
├── build/、install/、log/
└── build_mapping_framework/、install_mapping_framework/、log_mapping_framework/
```

目标环境是 Ubuntu 22.04、ROS 2 Humble。以下命令均在宿主机终端执行；不要在没有 ROS 2 的容器中执行。首次安装依赖和拉取锁定的外部仓库时，使用 `scripts/bootstrap.sh`，不要重复导入 `.repos` 中的包。

## 1. 全仓重新编译

已有工作区推荐显式指定所有目录，避免在仓库目录执行 `colcon` 时误用 `agt_mapping_framework/build`：

```bash
WS=/home/yangxuan/ros2_ws
REPO="$WS/src/agt_mapping_framework"

cd "$WS"
source /opt/ros/humble/setup.bash
# 提供 livox_ros_driver2、FAST-LIO2、PGO 等已安装的外部依赖。
source "$WS/install/setup.bash"

colcon build \
  --base-paths "$REPO" \
  --build-base "$WS/build" \
  --install-base "$WS/install" \
  --symlink-install \
  --cmake-clean-cache \
  --event-handlers console_cohesion+
```

成功判据是最后出现 `Summary: 14 packages finished`，且没有 `failed` 或 `aborted`。PCL 的可选 pcap/png 提示、OpenGL CMP0072 提示和 Qt5 `QWheelEvent::pos()` 弃用提示目前不阻断构建。

## 2. 更新默认运行 overlay

`run_mid360_mapping.sh` 和 `map_studio.sh` 会优先使用 `install_mapping_framework/setup.bash`。因此只更新 `install/` 后，默认入口仍可能加载旧程序。修改 Map Studio、在线建图编排或精修工具后，再执行：

```bash
cd "$WS"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

colcon --log-base "$WS/log_mapping_framework" build \
  --base-paths "$REPO" \
  --build-base "$WS/build_mapping_framework" \
  --install-base "$WS/install_mapping_framework" \
  --packages-select \
    agt_map_studio \
    agt_mapping_bringup \
    agt_pcd2grid_exporter \
    agt_map_refinement_core \
  --allow-overriding \
    agt_map_studio \
    agt_mapping_bringup \
    agt_pcd2grid_exporter \
    agt_map_refinement_core \
  --cmake-clean-cache \
  --event-handlers console_cohesion+
```

确认实际解析到新 overlay：

```bash
source "$WS/install_mapping_framework/setup.bash"
ros2 pkg prefix agt_map_studio
ros2 pkg prefix agt_mapping_bringup
ros2 pkg prefix agt_pcd2grid_exporter
ros2 pkg prefix agt_map_refinement_core
```

四个结果都应位于 `$WS/install_mapping_framework/`。

## 3. 自动化测试

### 全仓测试

```bash
cd "$WS"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

colcon test \
  --base-paths "$REPO" \
  --build-base "$WS/build" \
  --install-base "$WS/install" \
  --event-handlers console_cohesion+ \
  --return-code-on-test-failure
```

`build/` 可能还含同一 ROS 工作区中其他仓库的历史结果，不建议直接对整个目录运行一次 `colcon test-result` 后据此判断本仓库。按包查看更准确：

```bash
for package in \
  agt_fastlio_backend agt_map_refinement_core agt_map_studio \
  agt_mapping_artifacts agt_mapping_bringup agt_mapping_exporter \
  agt_mid360_adapter agt_pcd2grid_exporter agt_pgo_backend; do
  colcon test-result --test-result-base "$WS/build/$package" --all
done
```

只复测本次主要改动包时：

```bash
source "$WS/install_mapping_framework/setup.bash"
colcon --log-base "$WS/log_mapping_framework" test \
  --base-paths "$REPO" \
  --build-base "$WS/build_mapping_framework" \
  --install-base "$WS/install_mapping_framework" \
  --packages-select \
    agt_map_studio \
    agt_mapping_bringup \
    agt_pcd2grid_exporter \
    agt_map_refinement_core \
  --event-handlers console_cohesion+ \
  --return-code-on-test-failure
```

还可运行不启动 ROS 节点的编排契约回归：

```bash
cd "$REPO"
./scripts/test_mapping_workflow.sh
```

## 4. 入口冒烟测试

这些命令不会连接雷达、不会启动建图，也不会创建输出目录：

```bash
cd "$REPO"

bash -n \
  scripts/map_studio.sh \
  scripts/review_mapping_output.sh \
  scripts/run_mid360_mapping.sh \
  scripts/run_mid360_mapping_review.sh \
  scripts/run_mid360_live_mapping.sh

# 无显示环境可加 QT_QPA_PLATFORM=offscreen。
QT_QPA_PLATFORM=offscreen ./scripts/map_studio.sh --help
QT_QPA_PLATFORM=offscreen ./scripts/map_studio.sh --version

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source "$WS/install_mapping_framework/setup.bash"
ros2 launch agt_mapping_bringup mapping_live_mid360.launch.py --show-args
ros2 run agt_map_refinement_core apply_map_refinement --help

./scripts/run_mid360_live_mapping.sh \
  /tmp/agt_mapping_live_dry_run \
  --dry-run --no-rviz --duration 1
```

在线模式 dry-run 应输出合法 JSON，其中包含 Livox 配置、主机/雷达 IP、`/livox/lidar`、`/livox/imu`、`raw_bag` 和 launch argv；`runtime_checked: false` 是正常的，表示没有启动硬件运行时。

已有录包时，再做离线预检：

```bash
./scripts/run_mid360_mapping.sh /path/to/mid360_bag --dry-run --no-rviz
./scripts/run_mid360_mapping_review.sh /path/to/mid360_bag /tmp/unused_output --dry-run --no-rviz
```

## 5. GUI、录包与实机验收

### Map Studio

推荐先完成建图，再让框架自动做一次 PCD→PGM 并进入轻量二维审核：

```bash
./scripts/run_mid360_mapping_review.sh \
  /path/to/mid360_bag \
  /path/to/new_output
```

映射成功并通过 artifact 校验后，脚本会先利用关键帧 patch 做跨帧静态持久性过滤，
再进行局部地面分类、二维栅格化和边缘闭合，并创建
`<OUTPUT>/map_review/base/{map.pgm,map.yaml}`，然后启动只加载二维栅格的编辑器。
它不会渲染源 PCD，因此比 `--package` 的三维模式节省显存和内存。人工修改后点击
`Confirm & Save 2D Map`；只有这一步才会写入
`<OUTPUT>/map_review/confirmed/review_status.yaml`。看到其中
`status: confirmed` 才表示人工确认完成。

如果建图已经完成，不重新回放录包：

```bash
./scripts/review_mapping_output.sh /path/to/completed_mapping_output
```

生成二维图但暂不打开编辑器：

```bash
./scripts/review_mapping_output.sh /path/to/completed_mapping_output --no-studio
```

以上审核流程只调用本仓库的 `agt_pcd2grid_exporter` 与 `agt_map_studio`，不需要
`agt_navigation_v3` 的源码、overlay 或可执行程序。

传统三维点云编辑入口仍可单独运行。在有桌面的终端运行：

```bash
cd "$REPO"
./scripts/map_studio.sh --package /path/to/map_package
```

至少人工检查以下流程：

1. 打开点云，框选、反选、删除、撤销和重做。
2. 导出精修规则，确认重新加载后选择/删除状态一致。
3. 打开二维地图，执行矩形擦除、障碍线、多边形填充和禁行区操作。
4. 保存并恢复 `studio_session.yaml`，确认下游阶段失效提示正确。
5. 发布前检查生成的 PCD、Nav2 地图、patch 和版本化地图目录。

若要让可选的大地图加载用例参与测试：

```bash
export AGT_MAP_STUDIO_TEST_PCD=/path/to/large_map.pcd
colcon test \
  --base-paths "$REPO" \
  --build-base "$WS/build" \
  --install-base "$WS/install" \
  --packages-select agt_map_studio \
  --return-code-on-test-failure
```

### 录包回放

输出必须是新目录或空目录：

```bash
./scripts/run_mid360_mapping.sh \
  /path/to/mid360_bag \
  /path/to/new_output \
  --no-rviz --rate 1.0

./scripts/verify_map_artifact.sh /path/to/new_output
```

通过判据不是进程退出码本身，而是终端出现 `[4/4] Artifact VERIFIED`，并且 `session.json` 同时满足 `status: completed` 和 `artifact_verified: true`。

### MID-360 实机

实机前先确认 Livox JSON 中的主机 IP 已配置在雷达网口，并使用独立 ROS domain：

```bash
./scripts/run_mid360_live_mapping.sh \
  /path/to/new_live_output \
  --no-rviz --domain-id 89

ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 \
  ros2 service call /mapping/session/finish std_srvs/srv/Trigger '{}'
```

验收时检查 `/livox/lidar` 和 `/livox/imu` 持续有数据、传感器断流能中止且不误导出、`raw_bag/` 可回放，以及最终 `map_package` 能通过校验。自动化测试和 dry-run 不能代替雷达联网、真实运动数据、地图闭环质量与 GUI 人工体验验收。

## 6. 2026-09-21 本机验证记录

- ROS 2 Humble 下全仓 14 个 package 重新配置并编译成功。
- 全仓实际测试用例共 143 项。标准无外部数据运行时 142 项通过、1 项按条件跳过、0 失败；随后使用既有大地图复测跳过项并通过，因此 143 项均已得到通过结果。
- 主要改动包：`agt_mapping_bringup` 75 项通过，`agt_map_refinement_core` 8 项通过，`agt_map_studio` 23 项全部得到通过结果。
- `mapping_live_mid360.launch.py --show-args`、在线 `--dry-run`、精修 CLI help、Studio help/version 和 3 秒离屏启动均通过。
- 482.297 秒、268,427 条消息的既有 MID-360 基准录包通过只读 `--dry-run` 预检，话题和 launch argv 解析正确；本轮没有重新执行完整回放。
- 使用已完成的 bunker 输出实际转换 674,244 点 PCD，生成 2112×1577 PGM；转换耗时 2.16 秒，峰值常驻内存约 49 MB。轻量二维审核模式完成离屏启动冒烟测试，且没有加载 PCD 到三维视图。
- 增强转换使用 333 个关键帧 patch：674,371 点中跨帧持久性保留 529,169 点、过滤 145,202 点；局部地面/边缘处理后生成 2058×1782 PGM，耗时约 6 秒、峰值常驻内存约 116 MB。该数字表示短时观测过滤量，不等同于已人工确认的“人员点数量”。
- 本轮没有连接 MID-360，也没有重新执行完整真实录包或人工 GUI 编辑，因此这些仍属于现场验收项。
