/**
   @author Shin'ichiro Nakaoka
*/

#include "ButtonGroup.h"

using namespace cnoid;

ButtonGroup::ButtonGroup(QObject* parent)
    : QButtonGroup(parent)
{
    connect(this, SIGNAL(buttonClicked(int)), this, SLOT(onButtonClicked(int)));
}


void ButtonGroup::onButtonClicked(int id)
{
    sigButtonClicked_(id);
}
