/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCROLLBAR_H
#define CNOID_BASE_SCROLLBAR_H

#include <cnoid/Signal>
#include <QScrollBar>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ScrollBar : public QScrollBar
{
    Q_OBJECT

public:
    ScrollBar(QWidget* parent = 0);
    ScrollBar(Qt::Orientation orientation, QWidget* parent = 0);

    SignalProxy<void(int)> sigValueChanged() {
        return sigValueChanged_;
    }

private Q_SLOTS:
    void onValueChanged(int value);

private:
    Signal<void(int)> sigValueChanged_;
    void initialize();
};
    

class CNOID_EXPORT DoubleScrollBar : public QScrollBar
{
    Q_OBJECT

public:
    DoubleScrollBar(QWidget* parent = 0);
    DoubleScrollBar(Qt::Orientation orientation, QWidget* parent = 0);

    double getMaximum() const { return QScrollBar::maximum() / resolution; }
    double getMinimum() const { return QScrollBar::minimum() / resolution; }
    double getPageStep() const { return QScrollBar::pageStep() / resolution; }
    double getSingleStep() const { return QScrollBar::singleStep() / resolution; }
    double getValue() const { return value() / resolution; }
        
    void setRange(double min, double max) {
        QScrollBar::setRange(min * resolution, max * resolution);
    }
        
    void setPageStep(double step) {
        QScrollBar::setPageStep(step * resolution);
    }
        
    void setSingleStep(double step) {
        QScrollBar::setSingleStep(step * resolution);
    }

    void setValue(double value) {
        QScrollBar::setValue(value * resolution);
    }

    SignalProxy<void(double)> sigValueChanged() {
        return sigValueChanged_;
    }

private Q_SLOTS:
    void onValueChanged(int value);

private:
    Signal<void(double)> sigValueChanged_;
    double resolution;

    void initialize();        

    // Original int version functions are disabled
    int maximum() const { return QScrollBar::maximum(); }
    int minimum() const { return QScrollBar::minimum(); }
    int pageStep() const { return QScrollBar::pageStep(); }
    int singleStep() const { return QScrollBar::singleStep(); }
    int value() const { return QScrollBar::value(); }
    void setMaximum(int max) { QScrollBar::setMaximum(max); }
    void setMinimum(int min) { QScrollBar::setMinimum(min); }
    void setPageStep(int step) { QScrollBar::setPageStep(step); }
    void setRange(int min, int max) { QScrollBar::setRange(min, max); }
    void setSingleStep(int step) { QScrollBar::setSingleStep(step); }
    void setValue(int value) { QScrollBar::setValue(value); }
};

}

#endif
