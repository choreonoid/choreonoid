#include "DoubleSpinBox.h"
#include <QKeyEvent>

using namespace cnoid;


DoubleSpinBox::DoubleSpinBox(QWidget* parent)
    : QDoubleSpinBox(parent)
{
    setKeyboardTracking(false);
    isSettingValueInternally = false;
    isUndoRedoKeyInputEnabled_ = false;
    isEnterKeyEventConsumptionEnabled_ = true;
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
        sigValueChanged_.emplace();
        connect(this, (void(QDoubleSpinBox::*)(double)) &QDoubleSpinBox::valueChanged,
                [this](double value){ onValueChanged(value); });
    }
    return *sigValueChanged_;
}


SignalProxy<void()> DoubleSpinBox::sigEditingFinished()
{
    if(!sigEditingFinished_){
        sigEditingFinished_.emplace();
        connect(this, &QDoubleSpinBox::editingFinished,
                [this](){ onEditingFinished(); });
    }
    return *sigEditingFinished_;
}


SignalProxy<void()> DoubleSpinBox::sigEditingFinishedWithValueChange()
{
    if(!sigEditingFinishedWithValueChange_){
        sigEditingFinishedWithValueChange_.emplace();
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
    // Handle Enter key consumption
    if(isEnterKeyEventConsumptionEnabled_){
        if(event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter){
            interpretText();
            QDoubleSpinBox::editingFinished();
            event->accept();
            return;
        }
    }

    // Handle Undo/Redo keys
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
