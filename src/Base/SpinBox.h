/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_GUIBASE_SPINBOX_H_INCLUDED
#define CNOID_GUIBASE_SPINBOX_H_INCLUDED

#include <cnoid/SignalProxy>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SpinBox : public QSpinBox
{
    Q_OBJECT

        public:
    SpinBox(QWidget* parent = 0);
                               
    inline SignalProxy< boost::signal<void(int)> > sigValueChanged() {
        return sigValueChanged_;
    }
    inline SignalProxy< boost::signal<void()> > sigEditingFinished() {
        return sigEditingFinished_;
    }

private Q_SLOTS:
    void onValueChanged(int value);
    void onEditingFinishded();

private:
    boost::signal<void(int)> sigValueChanged_;
    boost::signal<void()> sigEditingFinished_;
};

class CNOID_EXPORT DoubleSpinBox : public QDoubleSpinBox
{
    Q_OBJECT

        public:
    DoubleSpinBox(QWidget* parent = 0);
                               
    inline SignalProxy< boost::signal<void(double)> > sigValueChanged() {
        return sigValueChanged_;
    }
    inline SignalProxy< boost::signal<void()> > sigEditingFinished() {
        return sigEditingFinished_;
    }

private Q_SLOTS:
    void onValueChanged(double value);
    void onEditingFinishded();

private:
    boost::signal<void(double)> sigValueChanged_;
    boost::signal<void()> sigEditingFinished_;
};

}

#endif
