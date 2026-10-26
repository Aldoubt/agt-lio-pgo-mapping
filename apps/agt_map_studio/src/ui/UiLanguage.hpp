#pragma once

#include <QString>

class QWidget;

namespace agt_map_studio {

// Translates only visible text. Combo-box itemData, enum values, filenames,
// asset IDs and serialized contract values remain unchanged.
class UiLanguage {
public:
  static QString preferred_language();
  static void set_language(QWidget *root, const QString &language, bool persist = true);
  static QString display_text(const QString &source, const QString &language);
};

}  // namespace agt_map_studio
