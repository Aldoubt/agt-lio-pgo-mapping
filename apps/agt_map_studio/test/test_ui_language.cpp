#include "ui/UiLanguage.hpp"

#include <QApplication>
#include <QComboBox>
#include <QLabel>
#include <QWidget>

#include <gtest/gtest.h>

TEST(UiLanguage, TranslatesVisibleLabelsWithoutChangingInternalValues) {
  EXPECT_EQ(agt_map_studio::UiLanguage::display_text(QStringLiteral("Query Sets"),
                                                     QStringLiteral("zh_CN")),
            QStringLiteral("查询集"));
  EXPECT_EQ(agt_map_studio::UiLanguage::display_text(QStringLiteral("AUTO"),
                                                     QStringLiteral("zh_CN")),
            QStringLiteral("AUTO"));
  EXPECT_EQ(agt_map_studio::UiLanguage::display_text(QStringLiteral("UNKNOWN"),
                                                     QStringLiteral("zh_CN")),
            QStringLiteral("UNKNOWN"));
  EXPECT_EQ(agt_map_studio::UiLanguage::display_text(QStringLiteral("Query Sets"),
                                                     QStringLiteral("en")),
            QStringLiteral("Query Sets"));
  EXPECT_EQ(agt_map_studio::UiLanguage::display_text(QStringLiteral("Study result markers"),
                                                     QStringLiteral("zh_CN")),
            QStringLiteral("实验结果标记"));
  EXPECT_EQ(agt_map_studio::UiLanguage::display_text(QStringLiteral("Z height filter"),
                                                     QStringLiteral("zh_CN")),
            QStringLiteral("Z 高度过滤"));
  EXPECT_EQ(agt_map_studio::UiLanguage::display_text(
                QStringLiteral("Offline inspection only. Source geometry remains read-only. Block size, query frames, and candidate Top-K are independent parameters. Block PCDs support visualization and leakage exclusion; existing per-keyframe GLOBAL/Top-K/GICP remains the analysis engine."),
                QStringLiteral("zh_CN")),
            QStringLiteral("仅用于离线检查，源几何保持只读。分块大小、查询帧数和候选 Top-K 相互独立。分块 PCD 用于可视化和数据泄漏排除；分析仍使用现有的逐关键帧 GLOBAL/Top-K/GICP 引擎。"));
  EXPECT_EQ(agt_map_studio::UiLanguage::display_text(
                QStringLiteral("Source: %1%2\nWork dir: %3"), QStringLiteral("zh_CN")),
            QStringLiteral("数据源：%1%2\n工作目录：%3"));
}

TEST(UiLanguage, SwitchesWidgetsBothWaysAndPreservesContractValues) {
  qputenv("QT_QPA_PLATFORM", "offscreen");
  int argc = 1;
  char app_name[] = "test_ui_language";
  char *argv[] = {app_name, nullptr};
  QApplication application(argc, argv);
  QWidget root;
  QLabel label(QStringLiteral("Query Sets"), &root);
  QComboBox filter(&root);
  filter.addItem(QStringLiteral("False Accept"), QStringLiteral("FALSE_ACCEPT"));
  QLabel result_layer(QStringLiteral("Study result markers"), &root);
  filter.addItem(QStringLiteral("Rejected"), QStringLiteral("REJECTED"));

  agt_map_studio::UiLanguage::set_language(&root, QStringLiteral("zh_CN"), false);
  EXPECT_EQ(label.text(), QStringLiteral("查询集"));
  EXPECT_EQ(filter.itemText(0), QStringLiteral("错误接受"));
  EXPECT_EQ(filter.currentData().toString(), QStringLiteral("FALSE_ACCEPT"));
  EXPECT_EQ(filter.itemData(1).toString(), QStringLiteral("REJECTED"));
  EXPECT_EQ(result_layer.text(), QStringLiteral("实验结果标记"));

  agt_map_studio::UiLanguage::set_language(&root, QStringLiteral("en"), false);
  EXPECT_EQ(label.text(), QStringLiteral("Query Sets"));
  EXPECT_EQ(filter.itemText(0), QStringLiteral("False Accept"));
  EXPECT_EQ(filter.itemData(0).toString(), QStringLiteral("FALSE_ACCEPT"));
  EXPECT_EQ(result_layer.text(), QStringLiteral("Study result markers"));
}
