#include "LengthSpinBox.h"
#include "DisplayValueFormat.h"
#include <cmath>

using namespace cnoid;

LengthSpinBox::LengthSpinBox(QWidget* parent)
    : DoubleSpinBox(parent)
{
    initialize();
}


LengthSpinBox::LengthSpinBox(double meterMinimum, double meterMaximum, QWidget* parent)
    : DoubleSpinBox(parent)
{
    meterMinimum_ = meterMinimum;
    meterMaximum_ = meterMaximum;
    initialize();
    setRange(dvFormat->toDisplayLength(meterMinimum), dvFormat->toDisplayLength(meterMaximum));
}


void LengthSpinBox::initialize()
{
    dvFormat = DisplayValueFormat::master();

    dvFormatConnection =
        dvFormat->sigFormatChanged().connect([this](){ onFormatChanged(); });

    unit = dvFormat->lengthUnit();
    updateDecimals();
    if(!isAutoStepAdjustmentEnabled_){
        setSingleStep(dvFormat->lengthStep());
    }
}


void LengthSpinBox::setMeterRange(double minimum, double maximum)
{
    meterMinimum_ = minimum;
    meterMaximum_ = maximum;
    if(fixedUnit_){
        setRange(toFixedUnitLength(minimum), toFixedUnitLength(maximum));
    } else {
        setRange(dvFormat->toDisplayLength(minimum), dvFormat->toDisplayLength(maximum));
    }
    updateDecimals();
}
        
double LengthSpinBox::meterMaximum() const
{
    if(meterMaximum_){
        return *meterMaximum_;
    }
    if(fixedUnit_){
        return fromFixedUnitLength(maximum());
    }
    return dvFormat->toMeter(maximum());
}


double LengthSpinBox::meterMinimum() const
{
    if(meterMinimum_){
        return *meterMinimum_;
    }
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


void LengthSpinBox::setAutoStepAdjustmentEnabled(bool on)
{
    isAutoStepAdjustmentEnabled_ = on;
    if(on){
        setSingleStep(std::pow(10.0, -decimals()));
    }
}


void LengthSpinBox::updateDecimals()
{
    int currentUnit = fixedUnit_ ? *fixedUnit_ : dvFormat->lengthUnit();
    int newDecimals = fixedDecimals_ ? *fixedDecimals_ : dvFormat->lengthDecimals();

    // Limit decimals to what is needed to represent meterMinimum_
    if(meterMinimum_ && *meterMinimum_ > 0.0){
        double displayMin;
        if(fixedUnit_){
            displayMin = toFixedUnitLength(*meterMinimum_);
        } else {
            displayMin = dvFormat->toDisplayLength(*meterMinimum_);
        }
        if(displayMin > 0.0){
            int requiredDecimals = std::max(
                0, static_cast<int>(std::ceil(-std::log10(displayMin) - 1e-9)));
            newDecimals = std::min(newDecimals, requiredDecimals);
        }
    }

    if(minMeterDecimals){
        int minDecimals;
        if(currentUnit == DisplayValueFormat::Meter){
            minDecimals = *minMeterDecimals;
        } else if(currentUnit == DisplayValueFormat::Millimeter){
            minDecimals = std::max(0, *minMeterDecimals - 3);
        } else { // Kilometer
            minDecimals = *minMeterDecimals + 3;
        }
        newDecimals = std::max(newDecimals, minDecimals);
    }

    if(newDecimals != decimals()){
        setDecimals(newDecimals);
        if(isAutoStepAdjustmentEnabled_){
            setSingleStep(std::pow(10.0, -newDecimals));
        }
        updateMinimumRange();
    }
}


void LengthSpinBox::updateMinimumRange()
{
    if(!meterMinimum_ || *meterMinimum_ <= 0.0){
        return;
    }
    int currentDecimals = this->decimals();
    double smallestDisplayValue = std::pow(10.0, -currentDecimals);
    double displayMin;
    if(fixedUnit_){
        displayMin = toFixedUnitLength(*meterMinimum_);
    } else {
        displayMin = dvFormat->toDisplayLength(*meterMinimum_);
    }
    setMinimum(std::max(displayMin, smallestDisplayValue));
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
    double meterMin = meterMinimum_ ? *meterMinimum_ : minimum() * oldRatio;
    double meterMax = meterMaximum_ ? *meterMaximum_ : maximum() * oldRatio;
    double meterVal = value() * oldRatio;

    updateDecimals();
    if(!isAutoStepAdjustmentEnabled_){
        if(meterSingleStep && !dvFormat->isLengthStepForcedMode()){
            setSingleStep(dvFormat->toDisplayLength(*meterSingleStep));
        } else {
            setSingleStep(dvFormat->lengthStep());
        }
    }
    if(newUnit != unit){
        // Convert saved meter values to new display unit
        setRange(dvFormat->toDisplayLength(meterMin), dvFormat->toDisplayLength(meterMax));
        setValue(dvFormat->toDisplayLength(meterVal));
        unit = newUnit;
        updateMinimumRange();
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
    if(!isAutoStepAdjustmentEnabled_){
        if(meterSingleStep){
            setSingleStep(toFixedUnitLength(*meterSingleStep));
        } else {
            setSingleStep(dvFormat->lengthStep());
        }
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

