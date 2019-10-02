#ifndef CNOID_BASE_STRING_LIST_COMBO_BOX_H
#define CNOID_BASE_STRING_LIST_COMBO_BOX_H

#include <QComboBox>
#include "exportdecl.h"

namespace cnoid {

/**
   \note This combo box uses the first element of the string list as the index of the selected item to
   exchange all the information required for the combo box as a form of a string list.
*/
class CNOID_EXPORT StringListComboBox : public QComboBox
{
    Q_OBJECT

    Q_PROPERTY(QStringList labels READ labels WRITE setLabels USER true)

public:
    StringListComboBox(QWidget* widget = 0);

    QStringList labels() const;
    void setLabels(QStringList labels);
};

}

#endif
