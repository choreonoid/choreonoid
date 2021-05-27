#ifndef CNOID_BASE_SPINBOX_H
#define CNOID_BASE_SPINBOX_H

#include <cnoid/Signal>
#include <QSpinBox>
#include <cnoid/DoubleSpinBox> // For backward compatibility
#include <cnoid/stdx/optional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SpinBox : public QSpinBox
{
public:
    SpinBox(QWidget* parent = nullptr);
                               
    SignalProxy<void(int)> sigValueChanged();
    SignalProxy<void()> sigEditingFinished();

private:
    stdx::optional<Signal<void(int)>> sigValueChanged_;
    stdx::optional<Signal<void()>> sigEditingFinished_;
};

}

#endif
