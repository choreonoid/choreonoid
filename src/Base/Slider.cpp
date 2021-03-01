/**
   @author Shin'ichiro NAKAOKA
*/

#include "Slider.h"

using namespace cnoid;

Slider::Slider(QWidget* parent)
    : QSlider(parent)
{

}


Slider::Slider(Qt::Orientation orientation, QWidget* parent)
    : QSlider(orientation, parent)
{

}


SignalProxy<void(int)> Slider::sigValueChanged()
{
    if(!sigValueChanged_){
        stdx::emplace(sigValueChanged_);
        connect(this, &QSlider::valueChanged,
                [this](int value){ (*sigValueChanged_)(value); });
    }
    return *sigValueChanged_;
}


SignalProxy<void()> Slider::sigSliderPressed()
{
    if(!sigSliderPressed_){
        stdx::emplace(sigSliderPressed_);
        connect(this, &QSlider::sliderPressed,
                [this](){ (*sigSliderPressed_)(); });
    }
    return *sigSliderPressed_;
}


SignalProxy<void()> Slider::sigSliderReleased()
{
    if(!sigSliderReleased_){
        stdx::emplace(sigSliderReleased_);
        connect(this, &QSlider::sliderReleased,
                [this](){ (*sigSliderReleased_)(); });
    }
    return *sigSliderReleased_;
}
