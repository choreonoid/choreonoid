#ifndef CNOID_BASE_SLIDER_H
#define CNOID_BASE_SLIDER_H

#include <cnoid/Signal>
#include <QSlider>
#include <cnoid/stdx/optional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Slider : public QSlider
{
public:
    Slider(QWidget* parent = nullptr);
    Slider(Qt::Orientation orientation, QWidget* parent = nullptr);
        
    SignalProxy<void(int)> sigValueChanged();
    SignalProxy<void()> sigSliderPressed();
    SignalProxy<void()> sigSliderReleased();

private:
    stdx::optional<Signal<void(int)>> sigValueChanged_;
    stdx::optional<Signal<void()>> sigSliderPressed_;
    stdx::optional<Signal<void()>> sigSliderReleased_;
};

}

#endif
