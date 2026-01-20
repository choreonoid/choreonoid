#ifndef CNOID_BASE_LENGTH_SPIN_BOX_H
#define CNOID_BASE_LENGTH_SPIN_BOX_H

#include "DoubleSpinBox.h"
#include "DisplayValueFormat.h"
#include "exportdecl.h"

namespace cnoid {

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

    //! Set minimum decimals in meter unit (e.g., 3 for 1mm precision)
    void setMinimumMeterDecimals(int decimals);

    //! Fix the length unit and decimals regardless of DisplayValueFormat settings
    void setFixedUnit(DisplayValueFormat::LengthUnit unit, int decimals);
    void clearFixedUnit();
    bool hasFixedUnit() const { return fixedUnit_.has_value(); }

private:
    double toFixedUnitLength(double meter) const;
    double fromFixedUnitLength(double displayLength) const;
    void onFormatChanged();
    void updateDecimals();

    DisplayValueFormat* dvFormat;
    ScopedConnection dvFormatConnection;
    int unit;
    std::optional<double> meterSingleStep;
    std::optional<int> minMeterDecimals;
    std::optional<int> fixedUnit_;
    std::optional<int> fixedDecimals_;
};

}

#endif

