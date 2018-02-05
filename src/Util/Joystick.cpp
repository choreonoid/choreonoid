#include "Joystick.h"
#include <cmath>

using namespace std;
using namespace cnoid;

double Joystick::getPosition(int axis, double threshold) const
{
    double pos = getPosition(axis);
    if(fabs(pos) < threshold){
        pos = 0.0;
    }
    return pos;
}
