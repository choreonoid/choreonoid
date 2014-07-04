/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_BASE_SPINBOX_H
#define CNOID_BASE_SPINBOX_H

#include <cnoid/Signal>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SpinBox : public QSpinBox
{
    Q_OBJECT

public:
    SpinBox(QWidget* parent = 0);
                               
    SignalProxy<void(int)> sigValueChanged() {
        return sigValueChanged_;
    }
    SignalProxy<void()> sigEditingFinished() {
        return sigEditingFinished_;
    }

private Q_SLOTS:
    void onValueChanged(int value);
    void onEditingFinishded();

private:
    Signal<void(int)> sigValueChanged_;
    Signal<void()> sigEditingFinished_;
};

class CNOID_EXPORT DoubleSpinBox : public QDoubleSpinBox
{
    Q_OBJECT

public:
    DoubleSpinBox(QWidget* parent = 0);
                               
    SignalProxy<void(double)> sigValueChanged() {
        return sigValueChanged_;
    }
    SignalProxy<void()> sigEditingFinished() {
        return sigEditingFinished_;
    }

private Q_SLOTS:
    void onValueChanged(double value);
    void onEditingFinishded();

private:
    Signal<void(double)> sigValueChanged_;
    Signal<void()> sigEditingFinished_;
};

}

#endif
