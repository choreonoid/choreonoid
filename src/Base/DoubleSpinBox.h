#ifndef CNOID_BASE_DOUBLE_SPINBOX_H
#define CNOID_BASE_DOUBLE_SPINBOX_H

#include <cnoid/Signal>
#include <QDoubleSpinBox>
#include <optional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT DoubleSpinBox : public QDoubleSpinBox
{
public:
    DoubleSpinBox(QWidget* parent = nullptr);

    void setUserInputEnabled(bool on) { setReadOnly(!on); }
    bool isUserInputEnabled() const { return !isReadOnly(); }
    void setUndoRedoKeyInputEnabled(bool on);
    bool isUndoRedoKeyInputEnabled() const { return isUndoRedoKeyInputEnabled_; }
    
    void setEnterKeyEventConsumptionEnabled(bool on) { isEnterKeyEventConsumptionEnabled_ = on; }
    bool isEnterKeyEventConsumptionEnabled() const { return isEnterKeyEventConsumptionEnabled_; }

    void setValue(double val);

    SignalProxy<void(double)> sigValueChanged();
    SignalProxy<void()> sigEditingFinished();
    SignalProxy<void()> sigEditingFinishedWithValueChange();

protected:
    void onValueChanged(double value);
    void onEditingFinished();
    virtual void keyPressEvent(QKeyEvent* event) override;

private:
    std::optional<Signal<void(double)>> sigValueChanged_;
    std::optional<Signal<void()>> sigEditingFinished_;
    std::optional<Signal<void()>> sigEditingFinishedWithValueChange_;
    bool isSettingValueInternally;
    bool isUndoRedoKeyInputEnabled_;
    bool isEnterKeyEventConsumptionEnabled_;
    bool valueChangedByLastUserInput;
};

}

#endif
