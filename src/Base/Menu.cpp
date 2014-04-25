/**
   @author Shin'ichiro NAKAOKA
*/

#include "Menu.h"

using namespace boost;
using namespace cnoid;


Menu::Menu(QWidget* parent)
    : QMenu(parent)
{
    initialize();
}


Menu::Menu(const QString& title, QWidget* parent)
    : QMenu(title, parent)
{
    initialize();
}


void Menu::initialize()
{
    sigTriggered_ = 0;
    sigAboutToShow_ = 0;
    sigAboutToHide_ = 0;
}


Menu::~Menu()
{
    if(sigTriggered_){
        delete sigTriggered_;
    }
    if(sigAboutToShow_){
        delete sigAboutToShow_;
    }
    if(sigAboutToHide_){
        delete sigAboutToHide_;
    }
}


SignalProxy< boost::signal<void(QAction*)> > Menu::sigTriggered()
{
    if(!sigTriggered_){
        sigTriggered_ = new boost::signal<void(QAction*)>();
        connect(this, SIGNAL(triggered(QAction*)), this, SLOT(onTriggered(QAction*)));
    }
    return *sigTriggered_;
}


void Menu::onTriggered(QAction* action)
{
    (*sigTriggered_)(action);
}


SignalProxy< boost::signal<void()> > Menu::sigAboutToShow()
{
    if(!sigAboutToShow_){
        sigAboutToShow_ = new boost::signal<void()>();
        connect(this, SIGNAL(aboutToShow()), this, SLOT(onAboutToShow()));
    }
    return *sigAboutToShow_;
}


void Menu::onAboutToShow()
{
    (*sigAboutToShow_)();
}


SignalProxy< boost::signal<void()> > Menu::sigAboutToHide()
{
    if(!sigAboutToHide_){
        sigAboutToHide_ = new boost::signal<void()>();
        connect(this, SIGNAL(aboutToHide()), this, SLOT(onAboutToHide()));
    }
    return *sigAboutToHide_;
}


void Menu::onAboutToHide()
{
    (*sigAboutToHide_)();
}
