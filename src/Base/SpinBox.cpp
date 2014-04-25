/**
   @author Shin'ichiro NAKAOKA
*/

#include "SpinBox.h"

using namespace cnoid;

SpinBox::SpinBox(QWidget* parent)
    : QSpinBox(parent)
{
    setKeyboardTracking(false);
    connect(this, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged(int)));
    connect(this, SIGNAL(editingFinished()), this, SLOT(onEditingFinishded()));
}
    

void SpinBox::onValueChanged(int value)
{
    sigValueChanged_(value);
}


void SpinBox::onEditingFinishded()
{
    sigEditingFinished_();
}


DoubleSpinBox::DoubleSpinBox(QWidget* parent)
    : QDoubleSpinBox(parent)
{
    setKeyboardTracking(false);
    connect(this, SIGNAL(valueChanged(double)), this, SLOT(onValueChanged(double)));
}
    

void DoubleSpinBox::onValueChanged(double value)
{
    sigValueChanged_(value);
}


void DoubleSpinBox::onEditingFinishded()
{
    sigEditingFinished_();
}
