#include "SpinBox.h"
#include <QKeyEvent>

using namespace cnoid;

SpinBox::SpinBox(QWidget* parent)
    : QSpinBox(parent)
{
    setKeyboardTracking(false);
}


SignalProxy<void(int)> SpinBox::sigValueChanged()
{
    if(!sigValueChanged_){
        stdx::emplace(sigValueChanged_);
        connect(this, (void(QSpinBox::*)(int)) &QSpinBox::valueChanged,
                [this](int value){ (*sigValueChanged_)(value); });
    }
    return *sigValueChanged_;
}


SignalProxy<void()> SpinBox::sigEditingFinished()
{
    if(!sigEditingFinished_){
        stdx::emplace(sigEditingFinished_);
        connect(this, &QSpinBox::editingFinished,
                [this](){ (*sigEditingFinished_)(); });
    }
    return *sigEditingFinished_;
}


DoubleSpinBox::DoubleSpinBox(QWidget* parent)
    : QDoubleSpinBox(parent)
{
    setKeyboardTracking(false);
    isSettingValueInternally = false;
    isUndoRedoKeyInputEnabled_ = false;
    valueChangedByLastUserInput = false;
}


void DoubleSpinBox::setUndoRedoKeyInputEnabled(bool on)
{
    isUndoRedoKeyInputEnabled_ = on;
    if(on){
        // Activate signal handlers
        sigValueChanged();
        sigEditingFinished();
    }
}


void DoubleSpinBox::setValue(double val)
{
    isSettingValueInternally = true;
    QDoubleSpinBox::setValue(val);
    isSettingValueInternally = false;
}


SignalProxy<void(double)> DoubleSpinBox::sigValueChanged()
{
    if(!sigValueChanged_){
        stdx::emplace(sigValueChanged_);
        connect(this, (void(QDoubleSpinBox::*)(double)) &QDoubleSpinBox::valueChanged,
                [this](double value){ onValueChanged(value); });
    }
    return *sigValueChanged_;
}


SignalProxy<void()> DoubleSpinBox::sigEditingFinished()
{
    if(!sigEditingFinished_){
        stdx::emplace(sigEditingFinished_);
        connect(this, &QDoubleSpinBox::editingFinished,
                [this](){ onEditingFinished(); });
    }
    return *sigEditingFinished_;
}


SignalProxy<void()> DoubleSpinBox::sigEditingFinishedWithValueChange()
{
    if(!sigEditingFinishedWithValueChange_){
        stdx::emplace(sigEditingFinishedWithValueChange_);
        sigEditingFinished();
    }
    return *sigEditingFinishedWithValueChange_;
}


void DoubleSpinBox::onValueChanged(double value)
{
    if(!isSettingValueInternally){
        valueChangedByLastUserInput = true;
    }
    (*sigValueChanged_)(value);
}


void DoubleSpinBox::onEditingFinished()
{
    (*sigEditingFinished_)();

    if(sigEditingFinishedWithValueChange_){
        if(valueChangedByLastUserInput){
            (*sigEditingFinishedWithValueChange_)();
        }
    }
    
    valueChangedByLastUserInput = false;
}


void DoubleSpinBox::keyPressEvent(QKeyEvent* event)
{
    bool isUndoOrRedoKey = false;
    if(isUndoRedoKeyInputEnabled_){
        if(event->key() == Qt::Key_Z && event->modifiers() & Qt::ControlModifier){
            isUndoOrRedoKey = true;
        }
    }
    if(isUndoOrRedoKey){
        if(valueChangedByLastUserInput){
            onEditingFinished();
        }
        QWidget::keyPressEvent(event);
    } else {
        QDoubleSpinBox::keyPressEvent(event);
    }
}
