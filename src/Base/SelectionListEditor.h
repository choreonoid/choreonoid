/**
   @author Shin'ichiro Nakaoka
*/

#include <QComboBox>
#include "exportdecl.h"

namespace cnoid {

/**
   This class is used for providing combo-box selection values in the tree views or table views.
*/
class CNOID_EXPORT SelectionListEditor : public QComboBox
{
    Q_OBJECT
        Q_PROPERTY(QStringList labels READ labels WRITE setLabels USER true);

public:
    SelectionListEditor(QWidget* widget = 0);

    QStringList labels() const;
    void setLabels(QStringList labels);
};
}
