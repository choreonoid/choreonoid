#include "DisplayedValueFormatManager.h"
#include "AppConfig.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class DisplayedValueFormatManager::Impl
{
public:
    int lengthUnit;
    int lengthDecimals;
    double lengthStep;
    int angleUnit;
    int angleDecimals;
    double angleStep;
    Signal<void()> sigFormatChanged;

    Impl();
};

}


DisplayedValueFormatManager* DisplayedValueFormatManager::instance()
{
    static DisplayedValueFormatManager manager;
    return &manager;
}


DisplayedValueFormatManager::DisplayedValueFormatManager()
{
    impl = new Impl;
}


DisplayedValueFormatManager::Impl::Impl()
{
    lengthUnit = Meter;
    lengthDecimals = 3;
    lengthStep = 0.001;
    angleUnit = Degree;
    angleDecimals = 1;
    angleStep = 1.0;
}


DisplayedValueFormatManager::~DisplayedValueFormatManager()
{
    delete impl;
}


int DisplayedValueFormatManager::lengthUnit() const
{
    return impl->lengthUnit;
}


void DisplayedValueFormatManager::setLengthUnit(int unitType)
{
    impl->lengthUnit = unitType;
}


int DisplayedValueFormatManager::lengthDecimals() const
{
    return impl->lengthDecimals;
}


void DisplayedValueFormatManager::setLengthDecimals(int decimals)
{
    impl->lengthDecimals = decimals;
}


double DisplayedValueFormatManager::lengthStep() const
{
    return impl->lengthStep;
}


void DisplayedValueFormatManager::setLengthStep(double step)
{
    impl->lengthStep = step;
}


int DisplayedValueFormatManager::angleUnit() const
{
    return impl->angleUnit;
}


void DisplayedValueFormatManager::setAngleUnit(int unitType)
{
    impl->angleUnit = unitType;
}


int DisplayedValueFormatManager::angleDecimals() const
{
    return impl->angleDecimals;
}


void DisplayedValueFormatManager::setAngleDecimals(int decimals)
{
    impl->angleDecimals = decimals;
}


double DisplayedValueFormatManager::angleStep() const
{
    return impl->angleStep;
}


void DisplayedValueFormatManager::setAngleStep(double step)
{
    impl->angleStep = step;
}


void DisplayedValueFormatManager::notifyFormatChange()
{
    impl->sigFormatChanged();
}


SignalProxy<void()> DisplayedValueFormatManager::sigFormatChanged()
{
    return impl->sigFormatChanged;
}


void DisplayedValueFormatManager::restoreConfiguration()
{
    Mapping& config = *AppConfig::archive()->findMapping("geometry");
    if(config){
        impl->lengthStep = config.get("length_step", impl->lengthStep);
    }
}
