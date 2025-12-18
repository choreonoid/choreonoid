#include "Slider.h"

using namespace cnoid;


Slider::Slider(QWidget* parent)
    : QSlider(parent)
{
    isUserInputEnabled_ = true;
}


Slider::Slider(Qt::Orientation orientation, QWidget* parent)
    : QSlider(orientation, parent)
{
    isUserInputEnabled_ = true;
}


SignalProxy<void(int)> Slider::sigValueChanged()
{
    if(!sigValueChanged_){
        sigValueChanged_.emplace();
        connect(this, &QSlider::valueChanged,
                [this](int value){ (*sigValueChanged_)(value); });
    }
    return *sigValueChanged_;
}


SignalProxy<void()> Slider::sigSliderPressed()
{
    if(!sigSliderPressed_){
        sigSliderPressed_.emplace();
        connect(this, &QSlider::sliderPressed,
                [this](){ (*sigSliderPressed_)(); });
    }
    return *sigSliderPressed_;
}


SignalProxy<void()> Slider::sigSliderReleased()
{
    if(!sigSliderReleased_){
        sigSliderReleased_.emplace();
        connect(this, &QSlider::sliderReleased,
                [this](){ (*sigSliderReleased_)(); });
    }
    return *sigSliderReleased_;
}


void Slider::keyPressEvent(QKeyEvent* event)
{
    if(isUserInputEnabled_){
        QSlider::keyPressEvent(event);
    }
}


void Slider::mousePressEvent(QMouseEvent* event)
{
    if(isUserInputEnabled_){
        QSlider::mousePressEvent(event);
    }
}


void Slider::wheelEvent(QWheelEvent* event)
{
    if(isUserInputEnabled_){
        QSlider::wheelEvent(event);
    }
}
