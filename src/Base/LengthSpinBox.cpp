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
    setDecimals(dvFormat->lengthDecimals());
    setSingleStep(dvFormat->lengthStep());
}


void LengthSpinBox::setMeterRange(double minimum, double maximum)
{
    if(dvFormat->isMeter()){
        setRange(minimum, maximum);
    } else if(dvFormat->isMillimeter()){
        setRange(minimum * 1000.0, maximum * 1000.0);
    }
}
        
double LengthSpinBox::meterMaximum() const
{
    if(dvFormat->isMeter()){
        return maximum();
    } else {
        return maximum() / 1000.0;
    }
}


double LengthSpinBox::meterMinimum() const
{
    if(dvFormat->isMeter()){
        return minimum();
    } else {
        return minimum() / 1000.0;
    }
}


void LengthSpinBox::setMeterSingleStep(double step)
{
    meterSingleStep = step;
    if(!dvFormat->isLengthStepForcedMode()){
        if(dvFormat->isMeter()){
            setSingleStep(step);
        } else {
            setSingleStep(step * 1000.0);
        }
    }
}


void LengthSpinBox::setMeterValue(double x)
{
    if(dvFormat->isMeter()){
        setValue(x);
    } else {
        setValue(x * 1000.0);
    }
}


double LengthSpinBox::meterValue() const
{
    if(dvFormat->isMeter()){
        return value();
    } else {
        return value() / 1000.0;
    }
}


void LengthSpinBox::onFormatChanged()
{
    int newUnit = dvFormat->lengthUnit();
    if(newUnit != unit){
        blockSignals(true);
        setDecimals(dvFormat->lengthDecimals());
        if(meterSingleStep && !dvFormat->isLengthStepForcedMode()){
            setSingleStep(*meterSingleStep);
        } else {
            setSingleStep(dvFormat->lengthStep());
        }
        if(dvFormat->isMeter()){
            setRange(minimum() / 1000.0, maximum() / 1000.0);
            setValue(value() / 1000.0);
        } else {
            setRange(minimum() * 1000.0, maximum() * 1000.0);
            setValue(value() * 1000.0);
        }
        blockSignals(false);
    }
}

