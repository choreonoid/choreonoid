/**
   @author Shizuko Hattori
*/

#include "Dial.h"

using namespace cnoid;


Dial::Dial(QWidget* parent)
    : QDial(parent)
{
    increasingValue = 0.0;
    preValue = 0;
    isSigValueChangedConnected = false;
}


SignalProxy<void(double)> Dial::sigValueChanged()
{
    if(!isSigValueChangedConnected){
        connect(this, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged(int)));
        isSigValueChangedConnected = true;
    }
    return sigValueChanged_;
}
        

double Dial::value()
{
    if(wrapping()){
        return increasingValue;
    }
    return QDial::value();
}


void Dial::setValue(double value)
{
    if(wrapping()){
        increasingValue = value;

        double dial_range = (double)maximum() - (double)minimum();
        int syo = value / dial_range;
        value -= dial_range * syo;
        if( value > maximum() ) value -= dial_range;
        if( value < minimum() ) value += dial_range;
    }

    QDial::setValue(value);
    preValue = value;
}


void Dial::onValueChanged(int value)
{
    if(wrapping()){
        double diff = (double)value - (double)preValue;
        double dial_range = (double)maximum() - (double)minimum();
        if( diff > dial_range/2.0 ){
            increasingValue -= (dial_range - diff);
        }else if( diff < -dial_range/2.0 ){
            increasingValue += (dial_range + diff);
        }else{
            increasingValue += diff;
        }
        sigValueChanged_(increasingValue);
    }else{
        sigValueChanged_(value);
    }

    preValue = value;
}
