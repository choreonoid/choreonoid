/**
   @author Shin'ichiro Nakaoka
*/

#include "ButtonGroup.h"

using namespace cnoid;


ButtonGroup::ButtonGroup(QObject* parent)
    : QButtonGroup(parent)
{
    sigButtonClickedConnected = false;
    sigButtonToggledConnected = false;
}


SignalProxy<void(int id)> ButtonGroup::sigButtonClicked()
{
    if(!sigButtonClickedConnected){
        connect(this, SIGNAL(buttonClicked(int)), this, SLOT(onButtonClicked(int)));
        sigButtonClickedConnected = true;
    }
    return sigButtonClicked_;
}
    

SignalProxy<void(int id, bool checked)> ButtonGroup::sigButtonToggled()
{
    if(!sigButtonToggledConnected){
        connect(this, SIGNAL(buttonToggled(int,bool)), this, SLOT(onButtonToggled(int,bool)));
        sigButtonToggledConnected = true;
    }
    return sigButtonToggled_;
}


void ButtonGroup::onButtonClicked(int id)
{
    sigButtonClicked_(id);
}


void ButtonGroup::onButtonToggled(int id, bool checked)
{
    sigButtonToggled_(id, checked);
}
