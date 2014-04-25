/**
   @author Shin'ichiro Nakaoka
*/

#include "Timer.h"

using namespace cnoid;

Timer::Timer(QObject* parent)
    : QTimer(parent)
{
    connect(this, SIGNAL(timeout()), this, SLOT(onTimeout()));
}


void Timer::onTimeout()
{
    sigTimeout_();
}
