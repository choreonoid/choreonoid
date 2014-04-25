/**
   @author Shin'ichiro Nakaoka
*/

#include "FloatingNumberBox.h"

using namespace cnoid;

FloatingNumberBox::FloatingNumberBox(QWidget* parent)
    : QLineEdit(parent)
{
    connect(this, SIGNAL(editingFinished()), this, SLOT(onEditingFinishded()));
    oldText = value_.string().c_str();
}
    

void FloatingNumberBox::onEditingFinishded()
{
    if(value_.set(text().toStdString())){
        sigValueChanged_(value_.value());
        oldText = text();
    } else {
        setText(oldText);
    }
}


void FloatingNumberBox::setValue(double v)
{
    value_ = v;
    setText(value_.string().c_str());
}


double FloatingNumberBox::value() const
{
    return value_.value();
}

