/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SOCKET_NOTIFIER_H
#define CNOID_BASE_SOCKET_NOTIFIER_H

#include <cnoid/Signal>
#include <QSocketNotifier>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SocketNotifier : public QSocketNotifier
{
    Q_OBJECT;

public:
    SocketNotifier(int socket, Type type, QObject* parent = 0);
                               
    SignalProxy<void(int socket)> sigActivated() {
        return sigActivated_;
    }

private Q_SLOTS:
    void onActivated(int socket);

private:
    Signal<void(int)> sigActivated_;
};
    
}

#endif
