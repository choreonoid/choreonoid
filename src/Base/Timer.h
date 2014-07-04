/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_TIMER_H
#define CNOID_BASE_TIMER_H

#include <cnoid/Signal>
#include <QTimer>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Timer : public QTimer
{
    Q_OBJECT

public:
    Timer(QObject* parent = 0);
                               
    SignalProxy<void()> sigTimeout() {
        return sigTimeout_;
    }

private Q_SLOTS:
    void onTimeout();

private:
    Signal<void()> sigTimeout_;
};
    
}

#endif
