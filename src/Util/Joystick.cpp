#include "Joystick.h"
#include <cmath>

using namespace std;
using namespace cnoid;

double Joystick::getDefaultPosition(int axis) const
{
    switch(axis){
    case L_TRIGGER_AXIS:
    case R_TRIGGER_AXIS:
        return -1.0;
    }

    return 0.0;
}


double Joystick::getPosition(int axis, double threshold) const
{
    double pos = getPosition(axis);
    if(fabs(pos) < threshold){
        pos = 0.0;
    }
    return pos;
}
