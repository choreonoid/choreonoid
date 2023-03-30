#include "Dial.h"

using namespace cnoid;


Dial::Dial(QWidget* parent)
    : QDial(parent)
{
    increasingValue = 0.0;
    preValue = 0;
    isUserInputEnabled_ = true;
}


SignalProxy<void(double)> Dial::sigValueChanged()
{
    if(!sigValueChanged_){
        stdx::emplace(sigValueChanged_);
        connect(this, (void(QDial::*)(double)) &QDial::valueChanged,
                [this](double value){ onValueChanged(value); });
    }
    return *sigValueChanged_;
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
        (*sigValueChanged_)(increasingValue);
    }else{
        (*sigValueChanged_)(value);
    }

    preValue = value;
}


void Dial::keyPressEvent(QKeyEvent* event)
{
    if(isUserInputEnabled_){
        QDial::keyPressEvent(event);
    }
}


void Dial::mousePressEvent(QMouseEvent* event)
{
    if(isUserInputEnabled_){
        QDial::mousePressEvent(event);
    }
}


void Dial::wheelEvent(QWheelEvent* event)
{
    if(isUserInputEnabled_){
        QDial::wheelEvent(event);
    }
}
