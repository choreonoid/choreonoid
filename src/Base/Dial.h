#ifndef CNOID_BASE_DIAL_H
#define CNOID_BASE_DIAL_H

#include <cnoid/Signal>
#include <QDial>
#include <cnoid/stdx/optional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Dial : public QDial
{
public:
    Dial(QWidget* parent = 0);
    
    void setUserInputEnabled(bool on) { isUserInputEnabled_ = on; }
    bool isUserInputEnabled() const { return isUserInputEnabled_; }

    void setValue(double value);
    double value();

    SignalProxy<void(double)> sigValueChanged();

protected:
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    virtual void wheelEvent(QWheelEvent* event) override;

private:
    void onValueChanged(int value);
    
    stdx::optional<Signal<void(double)>> sigValueChanged_;
    double increasingValue;
    int preValue;
    bool isUserInputEnabled_;
};

}

#endif
