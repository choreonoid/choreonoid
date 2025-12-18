#ifndef CNOID_BASE_SLIDER_H
#define CNOID_BASE_SLIDER_H

#include <cnoid/Signal>
#include <QSlider>
#include <optional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Slider : public QSlider
{
public:
    Slider(QWidget* parent = nullptr);
    Slider(Qt::Orientation orientation, QWidget* parent = nullptr);
        
    void setUserInputEnabled(bool on) { isUserInputEnabled_ = on; }
    bool isUserInputEnabled() const { return isUserInputEnabled_; }

    SignalProxy<void(int)> sigValueChanged();
    SignalProxy<void()> sigSliderPressed();
    SignalProxy<void()> sigSliderReleased();

protected:
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    virtual void wheelEvent(QWheelEvent* event) override;

private:
    std::optional<Signal<void(int)>> sigValueChanged_;
    std::optional<Signal<void()>> sigSliderPressed_;
    std::optional<Signal<void()>> sigSliderReleased_;
    bool isUserInputEnabled_;
};

}

#endif
