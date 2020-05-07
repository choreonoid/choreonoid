#ifndef CNOID_BASE_CHECK_BOX_ACTION_H
#define CNOID_BASE_CHECK_BOX_ACTION_H

#include "CheckBox.h"
//#include <cnoid/Signal>
#include <QWidgetAction>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT CheckBoxAction : public QWidgetAction
{
    //Q_OBJECT

public:
    CheckBoxAction(const QString& text, QObject* parent = Q_NULLPTR);

    CheckBox* checkBox() {
        return checkBox_;
    }

private:
    CheckBox* checkBox_;
};

}

#endif
