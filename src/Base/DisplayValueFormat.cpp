#include "DisplayValueFormat.h"
#include <cnoid/ValueTree>
#include "AppConfig.h"

using namespace std;
using namespace cnoid;

DisplayValueFormat DisplayValueFormat::instance_;


DisplayValueFormat::DisplayValueFormat()
{
    lengthUnit_ = Meter;
    meterDecimals_ = 3;
    millimeterDecimals_ = 1;
    meterStep_ = 0.001;
    millimeterStep_ = 1.0;
    isLengthDecimalsForcedMode_ = false;
    isLengthStepForcedMode_ = false;
    
    angleUnit_ = Degree;
    degreeDecimals_ = 1;
    radianDecimals_ = 2;
    degreeStep_ = 1.0;
    radianStep_ = 0.01;
    isAngleDecimalsForcedMode_ = false;
    isAngleStepForcedMode_ = false;
}


void DisplayValueFormat::setLengthDecimals(int decimals)
{
    if(lengthUnit_ == Meter){
        meterDecimals_ = decimals;
        millimeterDecimals_ = decimals + 3;
    } else {
        millimeterDecimals_ = decimals;
        decimals -= 3;
        if(decimals < 0){
            decimals = 0;
        }
        meterDecimals_ = decimals;
    }
}


void DisplayValueFormat::setLengthStep(double step)
{
    if(lengthUnit_ == Meter){
        meterStep_ = step;
        millimeterStep_ = step * 1000.0;
    } else {
        meterStep_ = step / 1000.0;
        millimeterStep_ = step;
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
    }
}
