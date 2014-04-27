/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_TIMER_H
#define CNOID_BASE_TIMER_H

#include <cnoid/SignalProxy>
#include <QTimer>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Timer : public QTimer
{
    Q_OBJECT

public:
    Timer(QObject* parent = 0);
                               
    SignalProxy< boost::signal<void()> > sigTimeout() {
        return sigTimeout_;
    }

private Q_SLOTS:
    void onTimeout();

private:
    boost::signal<void()> sigTimeout_;
};
    
}

#endif
