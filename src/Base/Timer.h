/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_BASE_TIMER_H_INCLUDED
#define CNOID_BASE_TIMER_H_INCLUDED

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
