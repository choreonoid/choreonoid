#include "LengthSpinBox.h"
#include "DisplayValueFormat.h"

using namespace cnoid;

LengthSpinBox::LengthSpinBox(QWidget* parent)
    : DoubleSpinBox(parent)
{
    dvFormat = DisplayValueFormat::master();

    dvFormatConnection =
        dvFormat->sigFormatChanged().connect([this](){ onFormatChanged(); });

    unit = dvFormat->lengthUnit();
    updateDecimals();
    setSingleStep(dvFormat->lengthStep());
}


void LengthSpinBox::setMeterRange(double minimum, double maximum)
{
    if(fixedUnit_){
        setRange(toFixedUnitLength(minimum), toFixedUnitLength(maximum));
    } else {
        setRange(dvFormat->toDisplayLength(minimum), dvFormat->toDisplayLength(maximum));
    }
}
        
double LengthSpinBox::meterMaximum() const
{
    if(fixedUnit_){
        return fromFixedUnitLength(maximum());
    }
    return dvFormat->toMeter(maximum());
}


double LengthSpinBox::meterMinimum() const
{
    if(fixedUnit_){
        return fromFixedUnitLength(minimum());
    }
    return dvFormat->toMeter(minimum());
}


void LengthSpinBox::setMeterSingleStep(double step)
{
    meterSingleStep = step;
    if(fixedUnit_){
        setSingleStep(toFixedUnitLength(step));
    } else if(!dvFormat->isLengthStepForcedMode()){
        setSingleStep(dvFormat->toDisplayLength(step));
    }
}


void LengthSpinBox::setMeterValue(double x)
{
    if(fixedUnit_){
        setValue(toFixedUnitLength(x));
    } else {
        setValue(dvFormat->toDisplayLength(x));
    }
}


double LengthSpinBox::meterValue() const
{
    if(fixedUnit_){
        return fromFixedUnitLength(value());
    }
    return dvFormat->toMeter(value());
}


void LengthSpinBox::setMinimumMeterDecimals(int decimals)
{
    minMeterDecimals = decimals;
    updateDecimals();
}


void LengthSpinBox::updateDecimals()
{
    int currentUnit = fixedUnit_ ? *fixedUnit_ : dvFormat->lengthUnit();
    // Use fixed decimals if set, otherwise use DisplayValueFormat
    int decimals = fixedDecimals_ ? *fixedDecimals_ : dvFormat->lengthDecimals();
    if(minMeterDecimals){
        int minDecimals;
        if(currentUnit == DisplayValueFormat::Meter){
            minDecimals = *minMeterDecimals;
        } else if(currentUnit == DisplayValueFormat::Millimeter){
            minDecimals = std::max(0, *minMeterDecimals - 3);
        } else { // Kilometer
            minDecimals = *minMeterDecimals + 3;
        }
        decimals = std::max(decimals, minDecimals);
    }
    setDecimals(decimals);
}


void LengthSpinBox::onFormatChanged()
{
    // If fixed unit mode, skip DisplayValueFormat changes
    if(fixedUnit_){
        return;
    }

    int newUnit = dvFormat->lengthUnit();
    blockSignals(true);

    // Save current values in meter before updateDecimals() to avoid rounding by setDecimals()
    double oldRatio = (unit == DisplayValueFormat::Meter) ? 1.0 :
                      (unit == DisplayValueFormat::Millimeter) ? 0.001 : 1000.0;
    double meterMin = minimum() * oldRatio;
    double meterMax = maximum() * oldRatio;
    double meterVal = value() * oldRatio;

    updateDecimals();
    if(meterSingleStep && !dvFormat->isLengthStepForcedMode()){
        setSingleStep(dvFormat->toDisplayLength(*meterSingleStep));
    } else {
        setSingleStep(dvFormat->lengthStep());
    }
    if(newUnit != unit){
        // Convert saved meter values to new display unit
        setRange(dvFormat->toDisplayLength(meterMin), dvFormat->toDisplayLength(meterMax));
        setValue(dvFormat->toDisplayLength(meterVal));
        unit = newUnit;
    }
    blockSignals(false);
}


void LengthSpinBox::setFixedUnit(DisplayValueFormat::LengthUnit fixedUnit, int decimals)
{
    fixedUnit_ = fixedUnit;
    fixedDecimals_ = decimals;
    dvFormatConnection.disconnect();  // Disconnect DisplayValueFormat change notification
    unit = fixedUnit;
    updateDecimals();
    if(meterSingleStep){
        setSingleStep(toFixedUnitLength(*meterSingleStep));
    } else {
        setSingleStep(dvFormat->lengthStep());
    }
}


void LengthSpinBox::clearFixedUnit()
{
    fixedUnit_.reset();
    fixedDecimals_.reset();
    unit = dvFormat->lengthUnit();
    dvFormatConnection =
        dvFormat->sigFormatChanged().connect([this](){ onFormatChanged(); });
    onFormatChanged();
}


double LengthSpinBox::toFixedUnitLength(double meter) const
{
    if(*fixedUnit_ == DisplayValueFormat::Meter){
        return meter;
    }
    if(*fixedUnit_ == DisplayValueFormat::Millimeter){
        return meter * 1000.0;
    }
    return meter / 1000.0;  // Kilometer
}


double LengthSpinBox::fromFixedUnitLength(double displayLength) const
{
    if(*fixedUnit_ == DisplayValueFormat::Meter){
        return displayLength;
    }
    if(*fixedUnit_ == DisplayValueFormat::Millimeter){
        return displayLength / 1000.0;
    }
    return displayLength * 1000.0;  // Kilometer
}

