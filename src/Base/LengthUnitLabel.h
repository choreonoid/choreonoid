#ifndef CNOID_BASE_LENGTH_UNIT_LABEL_H
#define CNOID_BASE_LENGTH_UNIT_LABEL_H

#include <cnoid/Signal>
#include <QLabel>
#include "exportdecl.h"

namespace cnoid {

class DisplayValueFormat;

class CNOID_EXPORT LengthUnitLabel : public QLabel
{
public:
    LengthUnitLabel(QWidget* parent = nullptr);

private:
    void onFormatChanged();

    DisplayValueFormat* dvFormat;
    ScopedConnection dvFormatConnection;
};

}

#endif
