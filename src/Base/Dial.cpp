/**
   @author Shizuko Hattori
*/

#include "Dial.h"

using namespace cnoid;

Dial::Dial(QWidget* parent)
    : QDial(parent)
{
    initialize();
}


void Dial::initialize()
{
    connect(this, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged(int)));
    pre_Value = 0;
    increasing_Value = 0;
}


double Dial::value(){
    if(wrapping()){
        return increasing_Value;
    }

    return QDial::value();
}


void Dial::setValue(double value)
{
    if(wrapping()){
        increasing_Value = value;

        double dial_range = (double)maximum() - (double)minimum();
        int syo = value / dial_range;
        value -= dial_range * syo;
        if( value > maximum() ) value -= dial_range;
        if( value < minimum() ) value += dial_range;
    }

    QDial::setValue(value);
    pre_Value = value;
}


void Dial::onValueChanged(int value)
{
    if(wrapping()){
        double diff = (double)value - (double)pre_Value;
        double dial_range = (double)maximum() - (double)minimum();
        if( diff > dial_range/2.0 ){
            increasing_Value -= (dial_range - diff);
        }else if( diff < -dial_range/2.0 ){
            increasing_Value += (dial_range + diff);
        }else{
            increasing_Value += diff;
        }
        sigValueChanged_(increasing_Value);
    }else{
        sigValueChanged_(value);
    }

    pre_Value = value;
}
