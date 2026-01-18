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
    setRange(dvFormat->toDisplayLength(minimum), dvFormat->toDisplayLength(maximum));
}
        
double LengthSpinBox::meterMaximum() const
{
    return dvFormat->toMeter(maximum());
}


double LengthSpinBox::meterMinimum() const
{
    return dvFormat->toMeter(minimum());
}


void LengthSpinBox::setMeterSingleStep(double step)
{
    meterSingleStep = step;
    if(!dvFormat->isLengthStepForcedMode()){
        setSingleStep(dvFormat->toDisplayLength(step));
    }
}


void LengthSpinBox::setMeterValue(double x)
{
    setValue(dvFormat->toDisplayLength(x));
}


double LengthSpinBox::meterValue() const
{
    return dvFormat->toMeter(value());
}


void LengthSpinBox::setMinimumMeterDecimals(int decimals)
{
    minMeterDecimals = decimals;
    updateDecimals();
}


void LengthSpinBox::updateDecimals()
{
    int decimals = dvFormat->lengthDecimals();
    if(minMeterDecimals){
        int minDecimals;
        if(dvFormat->lengthUnit() == DisplayValueFormat::Meter){
            minDecimals = *minMeterDecimals;
        } else if(dvFormat->lengthUnit() == DisplayValueFormat::Millimeter){
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
    int newUnit = dvFormat->lengthUnit();
    blockSignals(true);
    updateDecimals();
    if(meterSingleStep && !dvFormat->isLengthStepForcedMode()){
        setSingleStep(dvFormat->toDisplayLength(*meterSingleStep));
    } else {
        setSingleStep(dvFormat->lengthStep());
    }
    if(newUnit != unit){
        // Convert current values from old unit to meter, then to new display unit
        double oldRatio = (unit == DisplayValueFormat::Meter) ? 1.0 :
                          (unit == DisplayValueFormat::Millimeter) ? 0.001 : 1000.0;
        double meterMin = minimum() * oldRatio;
        double meterMax = maximum() * oldRatio;
        double meterVal = value() * oldRatio;
        setRange(dvFormat->toDisplayLength(meterMin), dvFormat->toDisplayLength(meterMax));
        setValue(dvFormat->toDisplayLength(meterVal));
        unit = newUnit;
    }
    blockSignals(false);
}

