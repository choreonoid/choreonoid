#ifndef CNOID_BASE_SPIN_BOX_H
#define CNOID_BASE_SPIN_BOX_H

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
                               
    void setUserInputEnabled(bool on) { setReadOnly(!on); }
    bool isUserInputEnabled() const { return !isReadOnly(); }

    void setEnterKeyEventConsumptionEnabled(bool on) { isEnterKeyEventConsumptionEnabled_ = on; }
    bool isEnterKeyEventConsumptionEnabled() const { return isEnterKeyEventConsumptionEnabled_; }

    SignalProxy<void(int)> sigValueChanged();
    SignalProxy<void()> sigEditingFinished();

protected:
    virtual void keyPressEvent(QKeyEvent* event) override;

private:
    stdx::optional<Signal<void(int)>> sigValueChanged_;
    stdx::optional<Signal<void()>> sigEditingFinished_;
    bool isEnterKeyEventConsumptionEnabled_;
};

}

#endif
