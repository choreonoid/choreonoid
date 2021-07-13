#ifndef CNOID_BASE_LENGTH_SPIN_BOX_H
#define CNOID_BASE_LENGTH_SPIN_BOX_H

#include "DoubleSpinBox.h"
#include "exportdecl.h"

namespace cnoid {

class DisplayValueFormat;

class CNOID_EXPORT LengthSpinBox : public DoubleSpinBox
{
public:
    LengthSpinBox(QWidget* parent = nullptr);

    void setMeterRange(double minimum, double maximum);
    double meterMaximum() const;
    double meterMinimum() const;

    //! This function overwrites the meterStep specified in DisplayValueFormat.
    void setMeterSingleStep(double step);
    
    void setMeterValue(double x);
    double meterValue() const;

private:
    void onFormatChanged();
    
    DisplayValueFormat* dvFormat;
    ScopedConnection dvFormatConnection;
    int unit;
    stdx::optional<double> meterSingleStep;
};

}

#endif

