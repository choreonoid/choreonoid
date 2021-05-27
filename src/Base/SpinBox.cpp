#include "SpinBox.h"

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
