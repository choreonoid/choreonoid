/**
   @author Shizuko Hattori
*/

#ifndef CNOID_BASE_DIAL_H
#define CNOID_BASE_DIAL_H

#include <cnoid/Signal>
#include <QDial>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Dial : public QDial
{
    Q_OBJECT

public:
    Dial(QWidget* parent = 0);
    void setValue(double value);
    double value();

    SignalProxy<void(double)> sigValueChanged();

private Q_SLOTS:
    void onValueChanged(int value);

private:
    Signal<void(double)> sigValueChanged_;
    double increasingValue;
    int preValue;
    bool isSigValueChangedConnected;
};

}

#endif
