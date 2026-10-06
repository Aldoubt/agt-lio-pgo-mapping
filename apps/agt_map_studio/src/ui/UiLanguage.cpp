#include "ui/UiLanguage.hpp"

#include <QAbstractButton>
#include <QAbstractItemView>
#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QDockWidget>
#include <QDialog>
#include <QEvent>
#include <QGroupBox>
#include <QHash>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QLibraryInfo>
#include <QLocale>
#include <QMainWindow>
#include <QMenu>
#include <QObject>
#include <QPlainTextEdit>
#include <QSettings>
#include <QStatusBar>
#include <QTableWidget>
#include <QTabWidget>
#include <QToolBar>
#include <QTranslator>
#include <QTreeWidget>
#include <QVariant>
#include <QWidget>

namespace agt_map_studio {
namespace {

constexpr auto kSourceText = "agt_language_source_text";
constexpr auto kLastText = "agt_language_last_text";
constexpr auto kSourceItems = "agt_language_source_items";
constexpr auto kLastItems = "agt_language_last_items";
constexpr int kSourceCellRole = Qt::UserRole + 701;
constexpr int kLastCellRole = Qt::UserRole + 702;
QString active_language;
QTranslator *qt_base_translator = nullptr;

const QHash<QString, QString> &chinese_catalog() {
  static const QHash<QString, QString> catalog = {
      {"AGT Map Studio", "AGT 地图工作台"}, {"File", "文件"}, {"Edit", "编辑"},
      {"View", "视图"}, {"Tools", "工具"}, {"Help", "帮助"}, {"Language", "语言"},
      {"Open PCD...", "打开 PCD…"}, {"Open Mapping / Map Package...", "打开建图包 / 地图包…"},
      {"Open Occupancy Map (map.yaml)...", "打开栅格地图（map.yaml）…"},
      {"Open Studio Session...", "打开工作台会话…"}, {"Save Studio Session", "保存工作台会话"},
      {"Save Camera View...", "保存相机视图…"}, {"Export", "导出"}, {"Quit", "退出"},
      {"Undo", "撤销"}, {"Redo", "重做"}, {"Delete Selected Points", "删除选中点"},
      {"Invert Selection", "反选"}, {"Clear Selection", "清除选择"},
      {"Select Height Band...", "选择高度范围…"}, {"Hide Deleted Points", "隐藏已删除点"},
      {"Isolate Selection", "仅显示选择项"}, {"Reset Camera", "重置视角"},
      {"Isometric View", "等轴测视图"}, {"Front View", "前视图"}, {"Top View", "俯视图"},
      {"Show Axis", "显示坐标轴"}, {"Dark Background", "深色背景"},
      {"Color by Z Height", "按 Z 高度着色"}, {"Solid Point Color", "单色点云"},
      {"3D Point Cloud", "3D 点云"}, {"2D Navigation Map", "2D 导航地图"},
      {"Point Size", "点大小"}, {"Increase Point Size", "增大点尺寸"},
      {"Decrease Point Size", "减小点尺寸"}, {"Default Point Size", "默认点尺寸"},
      {"Quick Occupancy Preview (studio projector)...", "快速栅格预览（工作台投影）…"},
      {"1. Apply 3D Refinement", "1. 应用 3D 地图精修"},
      {"2. Build Relocalization Assets", "2. 生成重定位分析资产"},
      {"3. Generate Navigation Layers", "3. 生成导航地图图层"},
      {"4. Apply 2D Patch", "4. 应用 2D 地图修改"},
      {"5. Publish Map Package", "5. 发布地图包"}, {"Run All Pending Steps", "运行所有待处理步骤"},
      {"Controls", "操作说明"}, {"Publish Workflow", "地图发布流程"},
      {"<b>1. 3D refinement</b>", "<b>1. 3D 地图精修</b>"},
      {"<b>2. Relocalization assets</b>", "<b>2. 重定位分析资产</b>"},
      {"<b>3. Navigation layers</b>", "<b>3. 导航地图图层</b>"},
      {"<b>4. 2D patch</b>", "<b>4. 2D 地图修改</b>"},
      {"<b>5. Publish map package</b>", "<b>5. 发布地图包</b>"},
      {"Write refinement.yaml and a binary refined map.pcd (apply_map_refinement when the source is a mapping package, otherwise the studio clean-map export).", "生成 refinement.yaml 和二进制精修地图 map.pcd（建图包数据源使用 apply_map_refinement，否则导出工作台清理后的地图）。"},
      {"build_relocalization_assets from the effective PCD.", "从当前生效的 PCD 构建重定位资产。"},
      {"pcd_to_nav_map + validate_nav_map (agt_navigation_v3).", "运行 pcd_to_nav_map 和 validate_nav_map（agt_navigation_v3）。"},
      {"patch_nav_map with the polygon_m edits drawn in the 2D view.", "将 2D 视图绘制的 polygon_m 修改应用到导航地图。"},
      {"create_map_package into maps/<map_id>/<version>; never overwrites an existing version.", "将 create_map_package 输出到 maps/<map_id>/<version>；不会覆盖已有版本。"},
      {"3D Edit", "3D 编辑"}, {"2D Edit", "2D 编辑"}, {"Navigate (N)", "浏览（N）"},
      {"Select (B)", "选择（B）"}, {"Delete (X)", "删除（X）"}, {" Tool: ", " 工具："},
      {"Rectangle (drag)", "矩形（拖动）"}, {"Polygon (click, double-click to close)", "多边形（单击添加，双击闭合）"},
      {"Sphere (click)", "球形（单击）"}, {"How points are selected in Select/Delete mode", "选择/删除模式下的点选方式"},
      {" r(m) ", " 半径（米）"}, {"Sphere selection radius", "球形选择半径"},
      {"Z window", "Z 高度范围"}, {"Limit rectangle/polygon selections to a height band", "将矩形/多边形选择限制在高度范围内"},
      {"Z height filter", "Z 高度过滤"},
      {"Filter the displayed cloud and limit rectangle/polygon selections to this Z range", "按此 Z 范围过滤点云显示，并限制矩形/多边形选择"},
      {"Display points:", "显示点数："}, {"Full", "全部"},
      {"Rendering sample cap only; selection, edits, and exports still use all source points", "仅限制渲染抽样数量；选择、编辑和导出仍使用全部源点云"},
      {"min ", "最小 "}, {"max ", "最大 "}, {"Width (m):", "宽度（米）："},
      {"Obstacle line width in meters", "障碍线宽度（米）"}, {"Close polygon", "闭合多边形"},
      {"Undo vertex", "撤销顶点"}, {"Fit (F)", "适配视图（F）"},
      {"Erase rect", "擦除矩形"}, {"Obstacle line", "障碍线"},
      {"Free polygon", "可通行多边形"}, {"Occupied polygon", "占用多边形"},
      {"Unknown polygon", "未知区域多边形"}, {"Forbidden zone", "禁行区"},
      {"Pan/zoom only", "仅平移/缩放"}, {"Drag: occupied -> free", "拖动：占用区域改为空闲"},
      {"Keep-out polygon (exported to keepout_zones.yaml)", "禁行多边形（导出到 keepout_zones.yaml）"},
      {"Publish Workflow Panel", "地图发布流程面板"},
      {"Spatial Confidence Evidence", "空间置信度证据"},
      {"No confidence derivative loaded", "尚未加载置信度派生数据"},
      {"Intent", "意图"}, {"Reason", "原因"}, {"Value [0,1]", "数值 [0,1]"},
      {"Custom FORCE_LOW value", "自定义 FORCE_LOW 数值"}, {"Apply to selected", "应用到选中项"},
      {"Restore Auto", "恢复 AUTO"}, {"Undo override", "撤销覆盖"}, {"Redo override", "重做覆盖"},
      {"Save Overrides YAML (intent only)", "保存覆盖 YAML（仅意图）"},
      {"Core rebuild reviewed derivative (new directory)", "由核心重建已审核派生数据（新目录）"},
      {"Relocalization MVP", "重定位分析"}, {"Open a validated mapping source package to begin.", "请先打开通过校验的建图源包。"},
      {"New assets root", "新资产目录"}, {"Block keyframes", "每块关键帧数"},
      {"Build new blocks", "生成新分块"}, {"Load blocks", "加载分块"},
      {"Select an immutable block-set directory", "选择不可变分块目录"},
      {"Schema v1 topology", "Schema v1 温室结构"}, {"Browse", "浏览"},
      {"Validate & show structure", "校验并显示结构"}, {"Edit in new draft", "在新草稿中编辑"},
      {"Keyframe", "关键帧"}, {"Map point (snap to keyframe)", "地图点（吸附到关键帧）"},
      {"Reviewed row position", "已审核行道位置"}, {"Query selection", "查询位置选择"},
      {"Keyframe ID", "关键帧 ID"}, {"Map point", "地图点"}, {"Pick on 3D map", "在 3D 地图上拾取"},
      {"confirmed physical row ID", "人工确认的物理行道 ID"}, {"Row ID", "行道 ID"},
      {"Along row s", "沿行距离 s"}, {"Query accumulation frames", "查询累积帧数"},
      {"Candidate Top-K", "候选 Top-K"},
      {"Installed default; select a trace-capable existing binary if required", "使用已安装默认项；必要时选择支持 trace 的现有程序"},
      {"Native localizer override", "原生定位器覆盖项"},
      {"Run offline query and save evidence", "运行离线查询并保存证据"},
      {"Saved immutable evidence directory", "已保存的不可变证据目录"}, {"Load", "加载"},
      {"Rank", "排名"}, {"Keyframe", "关键帧"}, {"Block", "分块"},
      {"Descriptor", "描述子"}, {"GICP", "GICP"}, {"Row", "行道"},
      {"Independent display layers", "独立显示图层"}, {"Structure / scene markers", "结构 / 场景标记"},
      {"Block bounds and centers", "分块边界与中心"}, {"Query at estimated pose", "估计位姿下的查询点"},
      {"Selected candidate", "选中候选项"}, {"Relocalization MVP Panel", "重定位分析面板"},
      {"Offline inspection only. Source geometry remains read-only. Block size, query frames, and candidate Top-K are independent parameters. Block PCDs support visualization and leakage exclusion; existing per-keyframe GLOBAL/Top-K/GICP remains the analysis engine.",
       "仅用于离线检查，源几何保持只读。分块大小、查询帧数和候选 Top-K 相互独立。分块 PCD 用于可视化和数据泄漏排除；分析仍使用现有的逐关键帧 GLOBAL/Top-K/GICP 引擎。"},
      {"Block set verified and displayed. Bounds are sparse display-only points.", "分块集合已校验并显示。边界为仅用于显示的稀疏点。"},
      {"Reference editor opened a new working revision derived from the frozen annotation. It restores this map's separate reference-line draft; proposed lines are not confirmed physical IDs.", "参考线编辑器已从冻结标注新建工作版本，并恢复此地图独立的参考线草稿；自动提议线不是已确认的实际行道编号。"},
      {"Reference editor opened with a separate working revision. First use generates map-bound row and aisle references; edits autosave separately and remain distinct from confirmed physical topology.", "参考线编辑器已打开独立工作版本。首次使用会生成绑定当前地图的垄线和行道参考线；编辑会单独自动保存，不会变成已确认的实际拓扑。"},
      {"Load evidence to inspect query provenance, block bindings, and candidate metrics.", "加载证据后可检查查询来源、分块绑定和候选指标。"},
      {"Study result markers", "实验结果标记"},
      {"Empirical classifications: Correct (green), False Accept (red), Rejected (yellow), Timeout (purple), No Data (gray). Not a probability layer.",
       "经验分类：正确（绿色）、错误接受（红色）、已拒绝（黄色）、超时（紫色）、无数据（灰色）。这不是概率图层。"},
      {"Project Browser", "项目浏览器"}, {"Project Browser Panel", "项目浏览器面板"},
      {"Open MapStudio Project", "打开 MapStudio 项目"}, {"Create MapStudio Project", "新建 MapStudio 项目"},
      {"Save / Rebind", "保存 / 重新绑定"}, {"Add Query Point", "添加查询点"},
      {"Sample Reviewed Rows", "从已审核行道采样"}, {"Manual Query", "手工查询"},
      {"Scene", "场景"}, {"Entry", "行首"}, {"Middle", "行中"}, {"Exit", "行尾"},
      {"Headland", "地头"}, {"Other / unknown", "其他 / 未知"},
      {"Query Set ID (letters, digits, _ or -)", "查询集 ID（字母、数字、下划线或连字符）"},
      {"Invalid Query Set ID", "查询集 ID 无效"}, {"Structure Query Set", "结构采样查询集"},
      {"Sampling Density", "采样密度"},
      {"Reviewed row sample ratios (entry / middle / exit)", "已审核行道采样比例（行首 / 行中 / 行尾）"},
      {"Select a draft Query Set", "选择一个查询集草稿"}, {"Frozen Query Set", "已冻结查询集"},
      {"New Relocalization Study", "新建重定位实验"}, {"Study ID", "实验 ID"},
      {"Study Query Windows", "实验查询窗口"},
      {"Query accumulation frames (subset of 1,3,5)", "查询累积帧数（从 1、3、5 中选择）"},
      {"Choose report parent directory", "选择报告父目录"},
      {"Open a project to restore source, structure, blocks, Query Sets, Studies and evidence.", "打开项目以恢复数据源、结构、分块、查询集、实验和证据。"},
      {"Project Assets", "项目资产"}, {"No project loaded", "尚未加载项目"},
      {"No topology annotation", "没有结构标注"},
      {"Validated source and blocks required", "需要通过校验的数据源和分块"},
      {"Confirm Review & Freeze", "确认人工审核并冻结"},
      {"Confirm manual structure review", "确认人工结构审核"},
      {"Annotation already frozen", "结构版本已冻结"}, {"Topology could not be read", "无法读取结构标注"},
      {"Source and topology required", "需要数据源和结构标注"},
      {"No source loaded", "尚未加载数据源"}, {"3D refinement", "3D 地图精修"},
      {"Relocalization assets", "重定位分析资产"}, {"Navigation layers", "导航地图图层"},
      {"2D patch", "2D 地图修改"}, {"Publish map package", "发布地图包"},
      {"Converter parameters (pcd_to_nav_map)", "地图转换参数（pcd_to_nav_map）"},
      {"Resolution (m)", "分辨率（米）"}, {"Margin (m)", "边界留白（米）"},
      {"Min points / cell", "每格最少点数"}, {"Max step (m)", "最大台阶高度（米）"},
      {"Max slope (deg)", "最大坡度（度）"}, {"Publish target", "发布目标"},
      {"Map root", "地图根目录"}, {"Map id", "地图 ID"}, {"Version", "版本"},
      {"Run all pending steps", "运行所有待处理步骤"}, {"Cancel", "取消"},
      {"Idle", "空闲"}, {"Run", "运行"}, {"Open output", "打开输出"},
      {"Source", "数据源"}, {"Structure", "结构"}, {"Blocks", "分块"},
      {"Query Sets", "查询集"}, {"Studies", "实验"}, {"Evidence", "证据"},
      {"Navigation Map", "导航地图"}, {"Routes", "路线"}, {"Project", "项目"},
      {"Save", "保存"}, {"Open Project", "打开项目"}, {"Create Project", "新建项目"},
      {"Freeze Query Set", "冻结查询集"}, {"New Query Set", "新建查询集"},
      {"New Study", "新建实验"}, {"Run Study", "运行实验"},
      {"Cancel Study", "取消实验"}, {"Export Study Report", "导出实验报告"},
      {"Filter", "筛选"}, {"All", "全部"}, {"False Accept", "错误接受"},
      {"Rejected", "已拒绝"}, {"Timeout", "超时"}, {"Show Query", "显示查询点"},
      {"Show Candidate", "显示候选点"}, {"Overlay", "叠加对比"},
      {"Enable / Disable Query", "启用 / 禁用查询"}, {"Enable Query", "启用查询"},
      {"Disable Query", "禁用查询"}, {"Delete Draft Query", "删除草稿查询"},
      {"Delete draft query", "删除草稿查询"}, {"Headlands", "地头区域"}, {"Scenes", "场景"},
      {"Frozen Query Set", "已冻结查询集"},
      {"Candidate KF", "候选关键帧"}, {"Candidate s", "候选沿行距离 s"},
      {"Global Score", "全局得分"}, {"GICP Status", "GICP 状态"}, {"Fitness", "匹配度"},
      {"XY Error", "XY 误差"}, {"Yaw Error", "航向角误差"}, {"Classification", "分类"},
      {"Runtime", "运行时间"}, {"Previous Candidate", "上一个候选"},
      {"Next Candidate", "下一个候选"}, {"All Scenes", "全部场景"},
      {"All Frame Counts", "全部帧数"}, {" frame(s)", " 帧"},
      {"English", "English"}, {"简体中文", "简体中文"},
      {"Open confidence derivative failed", "打开置信度派生数据失败"},
      {"Open geometry sidecar failed", "打开几何证据侧车失败"},
      {"Open Occupancy Map failed", "打开栅格地图失败"}, {"Open Session failed", "打开会话失败"},
      {"Export failed", "导出失败"}, {"Save failed", "保存失败"},
      {"Select height band", "选择高度范围"}, {"Z min (m)", "Z 最小值（米）"},
      {"Z max (m)", "Z 最大值（米）"}, {"points", "点"}, {"voxels", "体素"},
      {"File: %1 | Total: %2 | Rendered: %3 | Selected: %4 | Deleted: %5 | Visible: %6 | Mode: %7", "文件：%1 | 总点数：%2 | 渲染：%3 | 已选：%4 | 已删：%5 | 可见：%6 | 模式：%7"},
      {" | Z [%1, %2]", " | Z 范围 [%1, %2]"}, {" | FPS: %1", " | 帧率：%1"},
      {"Navigate", "浏览"}, {"Select", "选择"}, {"Delete", "删除"},
      {"Rect", "矩形"}, {"Polygon", "多边形"}, {"Sphere %1m", "球形 %1 米"},
      {"%1 pts, double-click or Enter to close, Esc cancels", "%1 个顶点；双击或按 Enter 闭合，按 Esc 取消"},
      {"click: sphere r=%1 m", "单击：球形半径 %1 米"},
      {"Z low: %1", "Z 最低：%1"}, {"high: %1", "最高：%1"},
      {"0.0 low", "0.0 低"}, {"1.0 high", "1.0 高"},
      {"Source: %1%2\nWork dir: %3", "数据源：%1%2\n工作目录：%3"},
      {" (mapping package)", "（建图包）"}, {" (bare PCD)", "（单独 PCD）"},
      {"No source loaded. Open a PCD or a mapping package.", "尚未加载数据源。请打开 PCD 或建图包。"},
      {"No 3D deletions yet: downstream steps use the source PCD.", "尚无 3D 删除操作：后续步骤将使用源 PCD。"},
      {"Refinement rules -> %1", "精修规则 -> %1"},
      {"No 2D edits yet: publish uses the generated navigation layers.", "尚无 2D 编辑：发布时将使用生成的导航图层。"},
      {"Patch YAML -> %1", "地图修改 YAML -> %1"}, {"Ready to publish.", "可以发布。"},
      {"Blocked: %1", "发布受阻：%1"}, {"fresh", "最新"}, {"missing", "缺失"},
      {"n/a", "不适用"},
      {"yes", "是"}, {"no", "否"}, {"not loaded", "未加载"},
      {"CURRENT", "CURRENT"}, {"STALE", "STALE"}, {"UNKNOWN", "UNKNOWN"},
  };
  return catalog;
}

QString translate(const QString &source, const QString &language) {
  if (language != QStringLiteral("zh_CN")) return source;
  const auto &catalog = chinese_catalog();
  auto exact = catalog.constFind(source);
  if (exact != catalog.cend()) return exact.value();
  // Preserve diagnostic values and translate stable UI prefixes only.
  static const QList<QPair<QString, QString>> prefixes = {
      {"Running: ", "正在运行："}, {"Running ", "正在运行 "},
      {"Failed", "失败"}, {"Saved session: ", "已保存会话："},
      {"Saved view: ", "已保存视图："}, {"Wrote ", "已写入 "},
      {"Exported clean map (preview artifact): ", "已导出清理地图（预览资产）："},
      {"Block build failed: ", "分块生成失败："}, {"Block preview failed: ", "分块预览失败："},
      {"Structure validation failed: ", "结构校验失败："}, {"Query job failed: ", "查询任务失败："},
      {"Source and blocks required", "需要数据源与分块"},
      {"Structure validated against this exact source:", "结构已针对当前数据源完成校验："},
      {"Click once on the 3D map.", "请在 3D 地图上单击一次。"},
  };
  for (const auto &entry : prefixes) {
    if (source.startsWith(entry.first)) return entry.second + source.mid(entry.first.size());
  }
  return source;
}

QString displayed_source(QObject *object, const QString &current,
                         const char *source_property = kSourceText,
                         const char *last_property = kLastText) {
  const QString last = object->property(last_property).toString();
  QString source = object->property(source_property).toString();
  if (!object->property(source_property).isValid() || current != last) source = current;
  object->setProperty(source_property, source);
  return source;
}

void translate_object(QObject *object, const QString &language) {
  if (!object) return;
  auto update = [object, &language](const QString &current, auto setter) {
    const QString source = displayed_source(object, current);
    const QString output = translate(source, language);
    setter(output);
    object->setProperty(kLastText, output);
  };
  if (auto *main = qobject_cast<QMainWindow *>(object))
    update(main->windowTitle(), [main](const QString &s) { main->setWindowTitle(s); });
  if (auto *dialog = qobject_cast<QDialog *>(object))
    update(dialog->windowTitle(), [dialog](const QString &s) { dialog->setWindowTitle(s); });
  if (auto *dock = qobject_cast<QDockWidget *>(object))
    update(dock->windowTitle(), [dock](const QString &s) { dock->setWindowTitle(s); });
  if (auto *menu = qobject_cast<QMenu *>(object))
    update(menu->title(), [menu](const QString &s) { menu->setTitle(s); });
  if (auto *toolbar = qobject_cast<QToolBar *>(object))
    update(toolbar->windowTitle(), [toolbar](const QString &s) { toolbar->setWindowTitle(s); });
  if (auto *label = qobject_cast<QLabel *>(object))
    update(label->text(), [label](const QString &s) { label->setText(s); });
  if (auto *button = qobject_cast<QAbstractButton *>(object))
    update(button->text(), [button](const QString &s) { button->setText(s); });
  if (auto *group = qobject_cast<QGroupBox *>(object))
    update(group->title(), [group](const QString &s) { group->setTitle(s); });
  if (auto *line = qobject_cast<QLineEdit *>(object)) {
    const QString source = displayed_source(line, line->placeholderText(),
        "agt_language_source_placeholder", "agt_language_last_placeholder");
    const QString output = translate(source, language);
    line->setPlaceholderText(output);
    line->setProperty("agt_language_last_placeholder", output);
  }
  if (auto *text = qobject_cast<QPlainTextEdit *>(object)) {
    const QString source = displayed_source(text, text->placeholderText(),
        "agt_language_source_placeholder", "agt_language_last_placeholder");
    const QString output = translate(source, language);
    text->setPlaceholderText(output);
    text->setProperty("agt_language_last_placeholder", output);
  }
  if (auto *combo = qobject_cast<QComboBox *>(object)) {
    QStringList current;
    for (int index = 0; index < combo->count(); ++index) current.push_back(combo->itemText(index));
    QStringList source = combo->property(kSourceItems).toStringList();
    const QStringList last = combo->property(kLastItems).toStringList();
    if (!combo->property(kSourceItems).isValid() || current != last) source = current;
    QStringList output;
    for (int index = 0; index < combo->count(); ++index) {
      const QString text = translate(source.value(index, current.value(index)), language);
      combo->setItemText(index, text);
      output.push_back(text);
    }
    combo->setProperty(kSourceItems, source);
    combo->setProperty(kLastItems, output);
  }
  if (auto *table = qobject_cast<QTableWidget *>(object)) {
    for (int column = 0; column < table->columnCount(); ++column) {
      auto *header = table->horizontalHeaderItem(column);
      if (!header) continue;
      QString source = header->data(kSourceCellRole).toString();
      if (source.isNull()) source = header->text();
      else if (header->text() != header->data(kLastCellRole).toString()) source = header->text();
      const QString output = translate(source, language);
      header->setData(kSourceCellRole, source);
      header->setData(kLastCellRole, output);
      header->setText(output);
    }
    for (int row = 0; row < table->rowCount(); ++row) {
      for (int column = 0; column < table->columnCount(); ++column) {
        auto *cell = table->item(row, column);
        if (!cell) continue;
        QString source = cell->data(kSourceCellRole).toString();
        if (source.isNull() || cell->text() != cell->data(kLastCellRole).toString()) source = cell->text();
        const QString output = translate(source, language);
        cell->setData(kSourceCellRole, source);
        cell->setData(kLastCellRole, output);
        cell->setText(output);
      }
    }
  }
  if (auto *tree = qobject_cast<QTreeWidget *>(object)) {
    for (int column = 0; column < tree->columnCount(); ++column) {
      auto *header = tree->headerItem();
      if (!header) continue;
      const QString text = header->text(column);
      QString source = header->data(column, kSourceCellRole).toString();
      if (source.isNull() || text != header->data(column, kLastCellRole).toString()) source = text;
      const QString output = translate(source, language);
      header->setData(column, kSourceCellRole, source);
      header->setData(column, kLastCellRole, output);
      header->setText(column, output);
    }
    const auto translate_items = [&language](QTreeWidgetItem *parent, const auto &self) -> void {
      if (!parent) return;
      for (int column = 0; column < parent->columnCount(); ++column) {
        const QString current = parent->text(column);
        QString source = parent->data(column, kSourceCellRole).toString();
        if (source.isNull() || current != parent->data(column, kLastCellRole).toString()) source = current;
        const QString output = translate(source, language);
        parent->setData(column, kSourceCellRole, source);
        parent->setData(column, kLastCellRole, output);
        parent->setText(column, output);
      }
      for (int index = 0; index < parent->childCount(); ++index) self(parent->child(index), self);
    };
    for (int index = 0; index < tree->topLevelItemCount(); ++index)
      translate_items(tree->topLevelItem(index), translate_items);
  }
  if (auto *tabs = qobject_cast<QTabWidget *>(object)) {
    QStringList source = tabs->property("agt_language_source_tabs").toStringList();
    QStringList last = tabs->property("agt_language_last_tabs").toStringList();
    QStringList current;
    for (int index = 0; index < tabs->count(); ++index) current.push_back(tabs->tabText(index));
    if (!tabs->property("agt_language_source_tabs").isValid() || current != last) source = current;
    QStringList output;
    for (int index = 0; index < tabs->count(); ++index) {
      const QString text = translate(source.value(index, current.value(index)), language);
      tabs->setTabText(index, text);
      output.push_back(text);
    }
    tabs->setProperty("agt_language_source_tabs", source);
    tabs->setProperty("agt_language_last_tabs", output);
  }
  if (auto *action = qobject_cast<QAction *>(object)) {
    update(action->text(), [action](const QString &s) { action->setText(s); });
    const QString tooltip_source = displayed_source(action, action->toolTip(),
        "agt_language_source_tooltip", "agt_language_last_tooltip");
    const QString tooltip_output = translate(tooltip_source, language);
    action->setToolTip(tooltip_output);
    action->setProperty("agt_language_last_tooltip", tooltip_output);
  }
  if (auto *widget = qobject_cast<QWidget *>(object)) {
    const QString tooltip_source = displayed_source(widget, widget->toolTip(),
        "agt_language_source_tooltip", "agt_language_last_tooltip");
    const QString tooltip_output = translate(tooltip_source, language);
    widget->setToolTip(tooltip_output);
    widget->setProperty("agt_language_last_tooltip", tooltip_output);
    const QString whats_source = widget->property("agt_language_source_whats_this").toString();
    if (!widget->property("agt_language_source_whats_this").isValid())
      widget->setProperty("agt_language_source_whats_this", widget->whatsThis());
    const QString whats = translate(widget->property("agt_language_source_whats_this").toString(), language);
    widget->setWhatsThis(whats);
  }
}

void apply_tree(QObject *root, const QString &language) {
  translate_object(root, language);
  for (QObject *child : root->children()) apply_tree(child, language);
}

void install_qt_base_translation(const QString &language) {
  if (!qApp) return;
  if (qt_base_translator) {
    qApp->removeTranslator(qt_base_translator);
    delete qt_base_translator;
    qt_base_translator = nullptr;
  }
  if (language != QStringLiteral("zh_CN")) return;
  auto *translator = new QTranslator(qApp);
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
  const QString directory = QLibraryInfo::path(QLibraryInfo::TranslationsPath);
#else
  const QString directory = QLibraryInfo::location(QLibraryInfo::TranslationsPath);
#endif
  if (translator->load(QStringLiteral("qtbase_zh_CN"), directory)) {
    qApp->installTranslator(translator);
    qt_base_translator = translator;
  } else {
    delete translator;
  }
}

class ShowTranslationFilter final : public QObject {
public:
  explicit ShowTranslationFilter(QObject *parent) : QObject(parent) {}
  bool eventFilter(QObject *watched, QEvent *event) override {
    if (event->type() == QEvent::Show && qobject_cast<QWidget *>(watched))
      apply_tree(watched, active_language);
    return false;
  }
};

void ensure_filter() {
  if (!qApp || qApp->property("agt_language_filter_installed").toBool()) return;
  auto *filter = new ShowTranslationFilter(qApp);
  qApp->installEventFilter(filter);
  qApp->setProperty("agt_language_filter_installed", true);
}

}  // namespace

QString UiLanguage::preferred_language() {
  QSettings settings(QStringLiteral("AGT"), QStringLiteral("MapStudio"));
  const QString saved = settings.value(QStringLiteral("ui/language")).toString();
  if (saved == QStringLiteral("en") || saved == QStringLiteral("zh_CN")) return saved;
  return QLocale::system().language() == QLocale::Chinese ? QStringLiteral("zh_CN")
                                                           : QStringLiteral("en");
}

QString UiLanguage::display_text(const QString &source, const QString &language) {
  return translate(source, language);
}

void UiLanguage::set_language(QWidget *root, const QString &language, bool persist) {
  active_language = language == QStringLiteral("zh_CN") ? QStringLiteral("zh_CN")
                                                         : QStringLiteral("en");
  ensure_filter();
  install_qt_base_translation(active_language);
  if (root) apply_tree(root, active_language);
  if (persist) {
    QSettings settings(QStringLiteral("AGT"), QStringLiteral("MapStudio"));
    settings.setValue(QStringLiteral("ui/language"), active_language);
  }
}

}  // namespace agt_map_studio
