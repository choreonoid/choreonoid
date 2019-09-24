#include "SignalIoConnectionMap.h"
#include "SignalIoDevice.h"
#include "Body.h"
#include "BodyCloneMap.h"

using namespace std;
using namespace cnoid;

SignalIoConnection::SignalIoConnection()
{
    for(int i=0; i < 2; ++i){
        signalIndex_[i] = 0;
    }
}


SignalIoConnection::SignalIoConnection(SignalIoDevice* outDevice, int outIndex, SignalIoDevice* inDevice, int inIndex)
{
    device_[Out] = outDevice;
    signalIndex_[Out] = outIndex;
    device_[In] = inDevice;
    signalIndex_[In] = inIndex;
}


SignalIoConnection::SignalIoConnection(const SignalIoConnection& org)
{
    for(int i=0; i < 2; ++i){
        device_[i] = org.device_[i];
        signalIndex_[i] = org.signalIndex_[i];
        bodyName_[i] = org.bodyName_[i];
        deviceName_[i] = org.deviceName_[i];
    }
}


SignalIoConnection::SignalIoConnection(const SignalIoConnection& org, BodyCloneMap& bodyCloneMap, bool doConnection)
{
    for(int i=0; i < 2; ++i){
        device_[i] = bodyCloneMap.getClone<SignalIoDevice>(org.device_[i]);
        signalIndex_[i] = org.signalIndex_[i];
        bodyName_[i] = org.bodyName_[i];
        deviceName_[i] = org.deviceName_[i];
    }
}


const std::string& SignalIoConnection::bodyName(IoType which) const
{
    if(auto device = device_[which]){
        if(auto body = device->body()){
            return body->name();
        }
    }
    return bodyName_[which];
}


const std::string& SignalIoConnection::deviceName(IoType which) const
{
    if(auto device = device_[which]){
        return device->name();
    } else {
        return deviceName_[which];
    }
}


void SignalIoConnection::setDevice(IoType which, SignalIoDevice* device)
{
    device_[which] = device;
    deviceName_[which] = device->name();
    if(auto body = device->body()){
        bodyName_[which] = body->name();
    }
}


void SignalIoConnection::setNames(IoType which, const std::string& bodyName, const std::string& deviceName)
{
    bodyName_[which] = bodyName;
    deviceName_[which] = deviceName;
    device_[which].reset();
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


void SignalIoConnectionMap::insert(int index, SignalIoConnection* connection)
{
    if(index >= connections_.size()){
        connections_.push_back(connection);
    } else {
        connections_.insert(connections_.begin() + index, connection);
    }
}


void SignalIoConnectionMap::removeConnectionsOfBody(Body* body)
{

}
