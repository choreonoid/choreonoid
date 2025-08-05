#include "SpinBox.h"
#include <QKeyEvent>

using namespace cnoid;


SpinBox::SpinBox(QWidget* parent)
    : QSpinBox(parent)
{
    setKeyboardTracking(false);
    isEnterKeyEventConsumptionEnabled_ = true; // Default to consuming Enter key
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


void SpinBox::keyPressEvent(QKeyEvent* event)
{
    if(isEnterKeyEventConsumptionEnabled_){
        if(event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter){
            event->accept();
            return;
        }
    }
    QSpinBox::keyPressEvent(event);
}
