#include "AngleUnitLabel.h"
#include "DisplayValueFormat.h"

using namespace cnoid;

AngleUnitLabel::AngleUnitLabel(QWidget* parent)
    : QLabel(parent)
{
    dvFormat = DisplayValueFormat::master();

    dvFormatConnection =
        dvFormat->sigFormatChanged().connect([this](){ onFormatChanged(); });

    updateText();
}


void AngleUnitLabel::onFormatChanged()
{
    updateText();
}


void AngleUnitLabel::updateText()
{
    if(dvFormat->isDegree()){
        setText("[deg]");
    } else {
        setText("[rad]");
    }
}
