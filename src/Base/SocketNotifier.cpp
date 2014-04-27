/**
   @author Shin'ichiro Nakaoka
*/

#include "SocketNotifier.h"

using namespace cnoid;

SocketNotifier::SocketNotifier(int socket, Type type, QObject* parent)
    : QSocketNotifier(socket, type, parent)
{
    connect(this, SIGNAL(activated(int)), this, SLOT(onActivated(int)));
}


void SocketNotifier::onActivated(int socket)
{
    sigActivated_(socket);
}
