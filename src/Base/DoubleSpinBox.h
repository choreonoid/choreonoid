#ifndef CNOID_BASE_DOUBLE_SPINBOX_H
#define CNOID_BASE_DOUBLE_SPINBOX_H

#include <cnoid/Signal>
#include <QDoubleSpinBox>
#include <cnoid/stdx/optional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT DoubleSpinBox : public QDoubleSpinBox
{
public:
    DoubleSpinBox(QWidget* parent = nullptr);

    void setUndoRedoKeyInputEnabled(bool on);
    bool isUndoRedoKeyInputEnabled() const { return isUndoRedoKeyInputEnabled_; }

    void setValue(double val);
        
    SignalProxy<void(double)> sigValueChanged();
    SignalProxy<void()> sigEditingFinished();
    SignalProxy<void()> sigEditingFinishedWithValueChange();

protected:
    void onValueChanged(double value);
    void onEditingFinished();
    virtual void keyPressEvent(QKeyEvent* event) override;

private:
    stdx::optional<Signal<void(double)>> sigValueChanged_;
    stdx::optional<Signal<void()>> sigEditingFinished_;
    stdx::optional<Signal<void()>> sigEditingFinishedWithValueChange_;
    bool isSettingValueInternally;
    bool isUndoRedoKeyInputEnabled_;
    bool valueChangedByLastUserInput;
};

}

#endif
