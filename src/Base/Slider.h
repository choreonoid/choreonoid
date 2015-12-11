/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SLIDER_H
#define CNOID_BASE_SLIDER_H

#include <cnoid/Signal>
#include <QSlider>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Slider : public QSlider
{
    Q_OBJECT

public:
    Slider(QWidget* parent = 0);
    Slider(Qt::Orientation orientation, QWidget* parent = 0);
        
    SignalProxy<void(int)> sigValueChanged() {
        return sigValueChanged_;
    }

private Q_SLOTS:
    void onValueChanged(int value);

private:
    Signal<void(int)> sigValueChanged_;

    void initialize();        
};

}

#endif
