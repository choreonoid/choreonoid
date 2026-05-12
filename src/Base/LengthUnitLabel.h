#ifndef CNOID_BASE_LENGTH_UNIT_LABEL_H
#define CNOID_BASE_LENGTH_UNIT_LABEL_H

#include <cnoid/DisplayValueFormat>
#include <cnoid/Signal>
#include <QLabel>
#include <optional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LengthUnitLabel : public QLabel
{
public:
    LengthUnitLabel(QWidget* parent = nullptr);

    //! Fix the length unit regardless of DisplayValueFormat settings
    void setFixedUnit(DisplayValueFormat::LengthUnit unit);
    void clearFixedUnit();

private:
    void onFormatChanged();

    DisplayValueFormat* dvFormat;
    ScopedConnection dvFormatConnection;
    std::optional<int> fixedUnit_;
};

}

#endif
