#ifndef CNOID_BASE_DISPLAYED_VALUE_FORMAT_MANAGER_H
#define CNOID_BASE_DISPLAYED_VALUE_FORMAT_MANAGER_H

#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT DisplayedValueFormatManager
{
public:
    static DisplayedValueFormatManager* instance();

    DisplayedValueFormatManager(const DisplayedValueFormatManager& org) = delete;

    enum LengthUnit { Meter, Millimeter };
    int lengthUnit() const;
    void setLengthUnit(int unitType);
    int lengthDecimals() const;
    void setLengthDecimals(int decimals);
    double lengthStep() const;
    void setLengthStep(double step);

    enum AngleUnit { Radian, Degree };
    int angleUnit() const;
    void setAngleUnit(int unitType);
    int angleDecimals() const;
    void setAngleDecimals(int decimals);
    double angleStep() const;
    void setAngleStep(double step);

    void notifyFormatChange();
    SignalProxy<void()> sigFormatChanged();

    void restoreConfiguration();

private:
    DisplayedValueFormatManager();
    ~DisplayedValueFormatManager();

    class Impl;
    Impl* impl;
};

}

#endif
