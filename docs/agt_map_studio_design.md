# `agt_map_studio` Round 1 Design

## 1. Scope

`agt_map_studio` 是一个独立的 offline map processing/viewer 工具，用于读取已有 `map_package/map.pcd` 并进行人工查看。它不属于 FAST-LIO2、PGO 或地图导出 pipeline，也不修改输入地图包。

Round 1 只实现：

- PCD 加载
- 3D 点云显示
- WASDQE 相机移动和 Shift 加速
- 鼠标旋转、平移、缩放
- 基础 Qt GUI、菜单、状态栏和 FPS

明确不实现：

- 删除点云或任何地图编辑
- PGM/占用栅格编辑
- 动态过滤
- AI 功能
- ROS 实时通信

数据流保持为：

```text
existing map_package/map.pcd (read-only)
                    |
                    v
              agt_map_studio
                    |
                    v
                 viewer
```

## 2. Repository scan

扫描目标：`/home/yangxuan/ros2_ws/src/agt_mapping_framework`。

当前结构是一个包含多个独立 ROS package 的 source workspace，没有根目录 `CMakeLists.txt` 或根 `package.xml`。现有 package 使用：

- ROS 2 Humble
- `ament_cmake`：C++/ROS interface package
- `ament_python`：Python orchestration/tool package
- `colcon`：workspace build

仓库当前 package boundary 明确要求 FAST-LIO2、PGO、HBA 和 `agt_navigation_v3` 保持外部/独立边界；本工具将放在新的 `apps/` 区域，不加入 `backends/`、`core/`、`exporters/` 或 `interfaces/`。

当前 artifact contract 的 source map package 包含：

```text
map_package/
├── manifest.yaml
├── metadata.yaml
├── map.pcd
├── poses.txt
├── poses_timed.txt
├── patches/
├── calibration.yaml
└── checksums.sha256
```

`agt_map_studio` 只读取 `map.pcd`，不覆盖、不重新打包、不更新 checksum。

## 3. Environment and dependency decision

检测结果：

| capability | detected result | decision |
| --- | --- | --- |
| ROS | ROS 2 Humble | use existing environment |
| Qt6 | not detected | do not install; do not require |
| Qt5 | Qt 5.15.3 | use Qt5 Widgets/OpenGL |
| PCL | PCL 1.12 headers/libraries and `pcl_visualization` available | use PCL IO/data types |
| Eigen | Eigen 3.4.0 | use for camera/math support where useful |
| OpenGL | system OpenGL/GLX libraries available | use through Qt OpenGL context |
| YAML-CPP | yaml-cpp 0.7.0 available | use only for camera config/view serialization |

Round 1 will not force-install Qt6 or replace the current environment. The package will prefer Qt5 because that is the available compatible GUI stack.

## 4. New package location and structure

新增 package：

```text
apps/agt_map_studio/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── config.yaml
├── src/
│   ├── main.cpp
│   ├── io/
│   │   ├── PCDLoader.cpp
│   │   └── PCDLoader.hpp
│   ├── viewer/
│   │   ├── PointCloudViewer.cpp
│   │   ├── PointCloudViewer.hpp
│   │   ├── CameraController.cpp
│   │   └── CameraController.hpp
│   └── ui/
│       ├── MainWindow.cpp
│       └── MainWindow.hpp
└── test/
    └── test_pcd_loader.cpp
```

Package build type：`ament_cmake`。

安装内容：

- executable：`map_viewer`
- runtime config：`share/agt_map_studio/config/config.yaml`

## 5. Component architecture

### 5.1 `PCDLoader`

接口输入为文件路径，输出为保留原始字段的 `pcl::PCLPointCloud2` 数据对象以及轻量渲染缓存。

职责：

- 调用 PCL PCD IO，支持 ASCII、binary 和 binary-compressed PCD。
- 验证 `x/y/z` 字段存在。
- 保留 `intensity` 和其他原始字段在完整 `PCLPointCloud2` 中，不将输入强制转换成只含 XYZ 的文件。
- 为渲染生成 XYZ float buffer；如果有 intensity，额外生成 intensity buffer。
- 输出 point count、字段列表、width/height 和 load error。

渲染缓存是显示优化，不是新的地图数据格式；Round 1 不写回 PCD。

### 5.2 `PointCloudViewer`

基于 Qt5 `QOpenGLWidget` 和 OpenGL buffer 渲染点云，避免逐点 Qt widget 绘制。

职责：

- 上传 XYZ buffer 到 GPU。
- 默认使用白色背景和按 Z 高度着色的点云（低处蓝色、高处红色）；保留 intensity buffer，但不得丢失。
- 支持 point size 调整。
- 支持白色/深色背景切换。
- 支持 Z 高度渐变着色与纯色模式切换，并显示 Z 范围图例。
- 绘制可开关的 XYZ 坐标轴。
- 使用 `QPainter` 在渲染完成后显示 FPS 和点数 overlay。
- 在无有效点云时显示清晰的错误/空状态，不崩溃。

百万级点云目标采用单次 buffer draw path；Round 1 不做八叉树、LOD、分块流式加载或编辑索引。

### 5.3 `CameraController`

维护相机位置、目标点、up 方向、orbit distance 和 projection 参数。

键盘控制：

| key | action |
| --- | --- |
| W/S | 前进/后退 |
| A/D | 左/右移动 |
| Q/E | 下降/上升 |
| Shift | 使用 `fast_speed` |
| R | reset camera |

鼠标控制：

| gesture | action |
| --- | --- |
| left drag | orbit/rotate |
| right drag | pan |
| wheel | zoom |

相机速度从 `config/config.yaml` 读取：

```yaml
camera:
  speed: 0.5
  fast_speed: 3.0
```

### 5.4 `MainWindow`

基于 Qt `QMainWindow`，包含：

- File/Open PCD：打开本地 PCD 文件。
- File/Save View：保存当前相机视角和 viewer 状态到 YAML，不修改地图。
- View/Reset Camera：按点云包围盒重置相机。
- View/Show Axis：切换坐标轴。
- View/Point Size：增加、减少或恢复点大小。
- View/Background：白色/深色背景。
- status bar：filename、point count、FPS、字段摘要。

命令行 PCD 路径优先于 GUI 空启动：

```bash
ros2 run agt_map_studio map_viewer --pcd /path/to/map.pcd
./map_viewer /path/to/map.pcd
```

## 6. Dependency boundary

目标直接依赖：

```text
ament_cmake
ament_index_cpp
Qt5::Widgets
Qt5::OpenGL
PCL common/io
Eigen3
OpenGL
yaml-cpp
```

不依赖：

- FAST-LIO2
- PGO/HBA
- `agt_mapping_core`
- `agt_mapping_backend_api`
- `agt_mapping_interfaces`
- `agt_navigation_v3`
- `rclcpp`/ROS runtime topics

包名虽然是 ROS package，Round 1 的 executable 是普通 offline GUI；不创建 ROS node，不订阅/发布 topic，不改变现有 mapping launch。

## 7. Data safety and map contract

`map.pcd` 打开流程必须是只读：

1. 检查输入路径和文件可读性。
2. PCL 加载到内存。
3. 显示字段、点数和输入文件名。
4. GUI 的 Save View 只写新的 view YAML。
5. 不调用现有 exporter，不写入 source `map_package`。

Round 1 不提供 Save Map、Delete Point、Clean Map 或任何覆盖原始 artifact 的操作。Round 2 的 `clean_map.pcd` 必须写入用户指定的新路径，并保留 parent artifact provenance。

## 8. Validation plan

### Build

```bash
cd /home/yangxuan/ros2_ws
colcon build --base-paths src/agt_mapping_framework/apps/agt_map_studio
```

### Smoke load

使用已有 artifact，例如：

```bash
ros2 run agt_map_studio map_viewer \
  --pcd /home/yangxuan/ros2_ws/experiments/artifacts/output/mid360_20260901_205036/map_package/map.pcd
```

验证：

- PCD 成功打开。
- 字段列表包含 `x y z intensity`（若输入存在 intensity）。
- point count 与 PCD header 一致。
- 点云可见、默认白色。
- FPS、point count 和 filename 出现在 GUI。
- WASDQE、Shift、鼠标左键/右键/滚轮生效。
- Reset Camera、Show Axis、Point Size、Background 和 Save View 生效。

### Scale target

已有验证 artifact 为约 674k/756k 点；将以该规模验证加载和显示，并以百万级点云作为 Round 1 性能目标。若 GPU/虚拟显示环境不支持 OpenGL，应将构建/加载与显示能力分别记录，不能把环境显示失败误判为 PCD 格式失败。

## 9. Round 2 boundary

Round 2 在本设计之上增加：

- Box Selection
- Delete key 删除选中点
- 红色删除点显示
- Undo/Redo
- `clean_map.pcd` 导出

这些功能必须继续位于 `apps/agt_map_studio`，不得回写 `backends/`、`core/`、`exporters/` 或现有 map pipeline。
