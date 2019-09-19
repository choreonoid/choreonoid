#ifndef CNOID_BODY_SIGNAL_IO_CONNECTION_MAP_H
#define CNOID_BODY_SIGNAL_IO_CONNECTION_MAP_H

#include <cnoid/Referenced>
#include <string>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class SignalIoDevice;
typedef ref_ptr<SignalIoDevice> SignalIoDevicePtr;

class Body;
class BodyCloneMap;

class CNOID_EXPORT SignalIoConnection : public Referenced
{
public:
    SignalIoConnection();
    SignalIoConnection(SignalIoDevice* outDevice, int outIndex, SignalIoDevice* inDevice, int inIndex);
    SignalIoConnection(const SignalIoConnection& org);
    SignalIoConnection(const SignalIoConnection& org, BodyCloneMap& bodyCloneMap, bool doConnection = false);

private:
    SignalIoDevicePtr outDevice_;
    int outIndex_;
    std::string outBodyName_;
    std::string outDeviceName_;

    SignalIoDevicePtr inDevice_;
    int inIndex_;
    std::string inBodyName_;
    std::string inDeviceName_;
};
typedef ref_ptr<SignalIoConnection> SignalIoConnectionPtr;

class CNOID_EXPORT SignalIoConnectionMap : public Referenced
{
public:
    SignalIoConnectionMap();
    SignalIoConnectionMap(const SignalIoConnectionMap& org);
    SignalIoConnectionMap(const SignalIoConnectionMap& org, BodyCloneMap& bodyCloneMap, bool doConnection = false);

    void removeConnectionsOfBody(Body* body);

private:
    std::vector<SignalIoConnectionPtr> connections_;
};

typedef ref_ptr<SignalIoConnectionMap> SignalIoConnectionMapPtr;

}

#endif
