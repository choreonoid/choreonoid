#include "SpinBox.h"
#include <QKeyEvent>

using namespace cnoid;


SpinBox::SpinBox(QWidget* parent)
    : QSpinBox(parent)
{
    setKeyboardTracking(false);
    isEnterKeyEventConsumptionEnabled_ = true;
}


SignalProxy<void(int)> SpinBox::sigValueChanged()
{
    if(!sigValueChanged_){
        sigValueChanged_.emplace();
        connect(this, (void(QSpinBox::*)(int)) &QSpinBox::valueChanged,
                [this](int value){ (*sigValueChanged_)(value); });
    }
    return *sigValueChanged_;
}


SignalProxy<void()> SpinBox::sigEditingFinished()
{
    if(!sigEditingFinished_){
        sigEditingFinished_.emplace();
        connect(this, &QSpinBox::editingFinished,
                [this](){ (*sigEditingFinished_)(); });
    }
    return *sigEditingFinished_;
}


void SpinBox::keyPressEvent(QKeyEvent* event)
{
    if(isEnterKeyEventConsumptionEnabled_){
        if(event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter){
            interpretText();
            QSpinBox::editingFinished();
            event->accept();
            return;
        }
    }
    QSpinBox::keyPressEvent(event);
}
