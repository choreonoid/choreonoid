#include "DisplayValueFormat.h"
#include <cnoid/ValueTree>
#include "AppConfig.h"

using namespace std;
using namespace cnoid;

ref_ptr<DisplayValueFormat> DisplayValueFormat::master_ = new DisplayValueFormat();


DisplayValueFormat::DisplayValueFormat()
{
    lengthUnit_ = Meter;
    meterDecimals_ = 3;
    millimeterDecimals_ = 1;
    kilometerDecimals_ = 3;
    meterStep_ = 0.001;
    millimeterStep_ = 1.0;
    kilometerStep_ = 0.001;
    isLengthDecimalsForcedMode_ = false;
    isLengthStepForcedMode_ = false;
    
    angleUnit_ = Degree;
    degreeDecimals_ = 1;
    radianDecimals_ = 2;
    degreeStep_ = 1.0;
    radianStep_ = 0.01;
    isAngleDecimalsForcedMode_ = false;
    isAngleStepForcedMode_ = false;

    coordinateSystem_ = RightHanded;
}


DisplayValueFormat::DisplayValueFormat(const DisplayValueFormat& org)
{
    lengthUnit_ = org.lengthUnit_;
    meterDecimals_ = org.meterDecimals_;
    millimeterDecimals_ = org.millimeterDecimals_;
    kilometerDecimals_ = org.kilometerDecimals_;
    meterStep_ = org.meterStep_;
    millimeterStep_ = org.millimeterStep_;
    kilometerStep_ = org.kilometerStep_;
    isLengthDecimalsForcedMode_ = org.isLengthDecimalsForcedMode_;
    isLengthStepForcedMode_ = org.isLengthStepForcedMode_;
    
    angleUnit_ = org.angleUnit_;
    degreeDecimals_ = org.degreeDecimals_;
    radianDecimals_ = org.radianDecimals_;
    degreeStep_ = org.degreeStep_;
    radianStep_ = org.radianStep_;
    isAngleDecimalsForcedMode_ = org.isAngleDecimalsForcedMode_;
    isAngleStepForcedMode_ = org.isAngleStepForcedMode_;

    coordinateSystem_ = org.coordinateSystem_;
}


void DisplayValueFormat::setLengthDecimals(int decimals)
{
    if(lengthUnit_ == Meter){
        meterDecimals_ = decimals;
    } else if(lengthUnit_ == Millimeter){
        millimeterDecimals_ = decimals;
    } else {
        kilometerDecimals_ = decimals;
    }
}


void DisplayValueFormat::setLengthStep(double step)
{
    if(lengthUnit_ == Meter){
        meterStep_ = step;
    } else if(lengthUnit_ == Millimeter){
        millimeterStep_ = step;
    } else {
        kilometerStep_ = step;
    }
}


void DisplayValueFormat::setAngleDecimals(int decimals)
{
    if(angleUnit_ == Degree){
        degreeDecimals_ = decimals;
        radianDecimals_ = decimals + 2;
    } else {
        radianDecimals_ = decimals;
        decimals -= 2;
        if(decimals < 0){
            decimals = 0;
        }
        degreeDecimals_ = decimals;
    }
}


void DisplayValueFormat::setAngleStep(double step)
{
    if(angleUnit_ == Degree){
        degreeStep_ = step;
        radianStep_ = step / 100.0;
    } else {
        radianStep_ = step;
        degreeStep_ = step * 100.0;
    }
}
    
        
void DisplayValueFormat::notifyFormatChange()
{
    sigFormatChanged_();
}


void DisplayValueFormat::restoreConfiguration()
{
    Mapping& config = *AppConfig::archive()->findMapping("display_value_format");
    if(config){
        meterStep_ = config.get("meter_step", meterStep_);
        millimeterStep_ = config.get("millimeter_step", millimeterStep_);
        kilometerStep_ = config.get("kilometer_step", kilometerStep_);
    }
}
