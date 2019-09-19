#include "SignalIoConnectionMap.h"
#include "SignalIoDevice.h"
#include "BodyCloneMap.h"

using namespace std;
using namespace cnoid;

SignalIoConnection::SignalIoConnection()
{
    outIndex_ = 0;
    inIndex_ = 0;
}


SignalIoConnection::SignalIoConnection(SignalIoDevice* outDevice, int outIndex, SignalIoDevice* inDevice, int inIndex)
    : outDevice_(outDevice),
      outIndex_(outIndex),
      inDevice_(inDevice),
      inIndex_(inIndex)
{

}


SignalIoConnection::SignalIoConnection(const SignalIoConnection& org)
{

}


SignalIoConnection::SignalIoConnection(const SignalIoConnection& org, BodyCloneMap& bodyCloneMap, bool doConnection)
{
    outDevice_ = bodyCloneMap.getClone<SignalIoDevice>(org.outDevice_);
    outIndex_ = org.outIndex_;
    inDevice_ = bodyCloneMap.getClone<SignalIoDevice>(org.inDevice_);
    inIndex_ = org.inIndex_;
}


SignalIoConnectionMap::SignalIoConnectionMap()
{

}


SignalIoConnectionMap::SignalIoConnectionMap(const SignalIoConnectionMap& org)
{

}


SignalIoConnectionMap::SignalIoConnectionMap(const SignalIoConnectionMap& org, BodyCloneMap& bodyCloneMap, bool doConnection)
{

}


void SignalIoConnectionMap::removeConnectionsOfBody(Body* body)
{

}


