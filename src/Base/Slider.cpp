/**
   @author Shin'ichiro NAKAOKA
*/

#include "Slider.h"

using namespace cnoid;

Slider::Slider(QWidget* parent)
    : QSlider(parent)
{
    initialize();
}


Slider::Slider(Qt::Orientation orientation, QWidget* parent)
    : QSlider(orientation, parent)
{
    initialize();
}


void Slider::initialize()
{
    connect(this, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged(int)));
}
    

void Slider::onValueChanged(int value)
{
    sigValueChanged_(value);
}

