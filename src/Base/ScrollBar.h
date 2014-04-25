/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_GUIBASE_SCROLLBAR_H_INCLUDED
#define CNOID_GUIBASE_SCROLLBAR_H_INCLUDED

#include <cnoid/SignalProxy>
#include <QScrollBar>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ScrollBar : public QScrollBar
{
    Q_OBJECT

        public:
    ScrollBar(QWidget* parent = 0);
    ScrollBar(Qt::Orientation orientation, QWidget* parent = 0);

    inline SignalProxy< boost::signal<void(int)> > sigValueChanged() {
        return sigValueChanged_;
    }

private Q_SLOTS:
    void onValueChanged(int value);

private:
    boost::signal<void(int)> sigValueChanged_;
    void initialize();
};
    

class CNOID_EXPORT DoubleScrollBar : public QScrollBar
{
    Q_OBJECT

        public:
    DoubleScrollBar(QWidget* parent = 0);
    DoubleScrollBar(Qt::Orientation orientation, QWidget* parent = 0);

    inline double getMaximum() const { return QScrollBar::maximum() / resolution; }
    inline double getMinimum() const { return QScrollBar::minimum() / resolution; }
    inline double getPageStep() const { return QScrollBar::pageStep() / resolution; }
    inline double getSingleStep() const { return QScrollBar::singleStep() / resolution; }
    inline double getValue() const { return value() / resolution; }
        
    inline void setRange(double min, double max) {
        QScrollBar::setRange(min * resolution, max * resolution);
    }
        
    inline void setPageStep(double step) {
        QScrollBar::setPageStep(step * resolution);
    }
        
    inline void setSingleStep(double step) {
        QScrollBar::setSingleStep(step * resolution);
    }

    inline void setValue(double value) {
        QScrollBar::setValue(value * resolution);
    }

    inline SignalProxy< boost::signal<void(double)> > sigValueChanged() {
        return sigValueChanged_;
    }

private Q_SLOTS:
    void onValueChanged(int value);

private:
    boost::signal<void(double)> sigValueChanged_;
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
