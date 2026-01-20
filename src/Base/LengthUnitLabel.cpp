#include "LengthUnitLabel.h"
#include "DisplayValueFormat.h"
#include <cnoid/Format>

using namespace cnoid;

LengthUnitLabel::LengthUnitLabel(QWidget* parent)
    : QLabel(parent)
{
    dvFormat = DisplayValueFormat::master();

    dvFormatConnection =
        dvFormat->sigFormatChanged().connect([this](){ onFormatChanged(); });

    setText(formatR("[{}]", dvFormat->lengthUnitSymbol()).c_str());
}


void LengthUnitLabel::onFormatChanged()
{
    setText(formatR("[{}]", dvFormat->lengthUnitSymbol()).c_str());
}


void LengthUnitLabel::setFixedUnit(DisplayValueFormat::LengthUnit fixedUnit)
{
    fixedUnit_ = fixedUnit;
    dvFormatConnection.disconnect();
    setText(formatR("[{}]", DisplayValueFormat::lengthUnitSymbol(fixedUnit)).c_str());
}


void LengthUnitLabel::clearFixedUnit()
{
    fixedUnit_.reset();
    dvFormatConnection =
        dvFormat->sigFormatChanged().connect([this](){ onFormatChanged(); });
    onFormatChanged();
}
