#pragma once
#include <QString>

namespace agt_map_studio {
// Shared display catalog; MainWindow remains the single language preference owner.
QString localized_ui_text(const QString &text, bool chinese);
}
