/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SLIDER_H
#define CNOID_BASE_SLIDER_H

#include <cnoid/Signal>
#include <QSlider>
#include <QDial>
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

class CNOID_EXPORT Dial : public QDial
{
    Q_OBJECT

public:
    Dial(QWidget* parent = 0);
    void setValue(double value);
    double value();

    SignalProxy<void(double)> sigValueChanged() {
        return sigValueChanged_;
    }

private Q_SLOTS:
    void onValueChanged(int value);

private:
    Signal<void(double)> sigValueChanged_;
    int pre_Value;
    double increasing_Value;

    void initialize();

};

}

#endif
