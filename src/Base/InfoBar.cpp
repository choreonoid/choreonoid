/**
   @author Shin'ichiro Nakaoka
*/

#include "InfoBar.h"
#include "View.h"
#include <QApplication>
#include <iostream>

using namespace std;
using namespace cnoid;


InfoBar* InfoBar::instance()
{
    static InfoBar* infoBar = new InfoBar();
    return infoBar;
}


InfoBar::InfoBar()
{
    indicatorBase = new QWidget(this);
    indicatorLayout = new QHBoxLayout();
    indicatorLayout->setContentsMargins(0, 0, 0, 0);
    indicatorLayout->setSpacing(0);
    indicatorBase->setLayout(indicatorLayout);
    addPermanentWidget(indicatorBase);
    
    currentIndicator = 0;

    connect(qApp, SIGNAL(focusChanged(QWidget*, QWidget*)),
            this, SLOT(onFocusChanged(QWidget*, QWidget*)));
}


InfoBar::~InfoBar()
{
    removeCurrentIndicator();
}


void InfoBar::notify(const char* message)
{
    showMessage(message, 5000);
}


void InfoBar::notify(const std::string& message)
{
    showMessage(QString(message.c_str()), 5000);
}


void InfoBar::notify(const QString& message)
{
    showMessage(message, 5000);
}


void InfoBar::setIndicator(QWidget* indicator)
{
    if(indicator != currentIndicator){
        removeCurrentIndicator();
        if(indicator){
            indicatorLayout->addWidget(indicator);
            indicator->show();
            connect(indicator, SIGNAL(destroyed(QObject*)),
                    this, SLOT(onIndicatorDestroyed(QObject*)));
            currentIndicator = indicator;
        }
    }
}


void InfoBar::removeCurrentIndicator()
{
    if(currentIndicator){
        indicatorLayout->removeWidget(currentIndicator);
        currentIndicator->hide();
        currentIndicator->setParent(0);
        currentIndicator->disconnect(SIGNAL(destroyed(QObject*)), this, SLOT(onIndicatorDestroyed(QObject*)));
        currentIndicator = 0;
    }
}    


void InfoBar::onIndicatorDestroyed(QObject* obj)
{
    if(obj == currentIndicator){
        currentIndicator = 0;
    }
}


void InfoBar::onFocusChanged(QWidget* old, QWidget* now)
{
    QWidget* widget = now;
    while(widget){
        View* view = dynamic_cast<View*>(widget);
        if(view){
            QWidget* indicator = view->indicatorOnInfoBar();
            if(indicator){
                setIndicator(indicator);
                break;
            }
        }
        widget = widget->parentWidget();
    }
}

