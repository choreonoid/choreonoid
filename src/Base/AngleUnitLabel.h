#ifndef CNOID_BASE_ANGLE_UNIT_LABEL_H
#define CNOID_BASE_ANGLE_UNIT_LABEL_H

#include <cnoid/Signal>
#include <QLabel>
#include "exportdecl.h"

namespace cnoid {

class DisplayValueFormat;

class CNOID_EXPORT AngleUnitLabel : public QLabel
{
public:
    AngleUnitLabel(QWidget* parent = nullptr);

private:
    void onFormatChanged();
    void updateText();

    DisplayValueFormat* dvFormat;
    ScopedConnection dvFormatConnection;
};

}

#endif
