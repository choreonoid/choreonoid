/**
   @author Shin'ichiro Nakaoka
*/

#include "ActionGroup.h"

using namespace cnoid;


ActionGroup::ActionGroup(QObject* parent)
    : QActionGroup(parent)
{
    sigHovered_ = 0;
    sigTriggered_ = 0;
}


ActionGroup::~ActionGroup()
{
    if(sigHovered_){
        delete sigHovered_;
    }
    if(sigTriggered_){
        delete sigTriggered_;
    }
}


SignalProxy<void(QAction*)> ActionGroup::sigHovered()
{    
    if(!sigHovered_){
        sigHovered_ = new Signal<void(QAction*)>();
        connect(this, SIGNAL(triggered(QAction*)), this, SLOT(onHovered(QAction*)));
    }
    return *sigHovered_;
}


void ActionGroup::onHovered(QAction* action)
{
    (*sigHovered_)(action);
}


SignalProxy<void(QAction*)> ActionGroup::sigTriggered()
{    
    if(!sigTriggered_){
        sigTriggered_ = new Signal<void(QAction*)>();
        connect(this, SIGNAL(triggered(QAction*)), this, SLOT(onTriggered(QAction*)));
    }
    return *sigTriggered_;
}


void ActionGroup::onTriggered(QAction* action)
{
    (*sigTriggered_)(action);
}
