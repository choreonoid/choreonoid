/**
   @author Shin'ichiro Nakaoka
*/

#include "Action.h"

using namespace cnoid;


Action::Action(QObject* parent)
    : QAction(parent)
{
    initialize();
}


Action::Action(const QString& text, QObject* parent)
    : QAction(text, parent)
{
    initialize();
}


Action::Action(const QIcon& icon, QObject* parent)
    : QAction(parent)
{
    setIcon(icon);
    initialize();
}


Action::Action(const QIcon& icon, const QString& text, QObject* parent)
    : QAction(icon, text, parent)
{
    initialize();
}


void Action::initialize()
{
    sigTriggered_ = 0;
    sigToggled_ = 0;
}


Action::~Action()
{
    if(sigTriggered_){
        delete sigTriggered_;
    }
    if(sigToggled_){
        delete sigToggled_;
    }
}


SignalProxy<void()> Action::sigTriggered()
{    
    if(!sigTriggered_){
        sigTriggered_ = new Signal<void(void)>();
        connect(this, SIGNAL(triggered(bool)), this, SLOT(onTriggered(bool)));
    }
    return *sigTriggered_;
}


void Action::onTriggered(bool checked)
{
    (*sigTriggered_)();
}


SignalProxy<void(bool)> Action::sigToggled()
{    
    if(!sigToggled_){
        sigToggled_ = new Signal<void(bool)>();
        connect(this, SIGNAL(toggled(bool)), this, SLOT(onToggled(bool)));
    }
    return *sigToggled_;
}


void Action::onToggled(bool checked)
{
    (*sigToggled_)(checked);
}
