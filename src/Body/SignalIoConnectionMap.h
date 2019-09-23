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

    enum { Out = 0, In = 1 };

    SignalIoDevice* outDevice(){ return device_[Out]; }
    int outSignalIndex(){ return signalIndex_[Out]; }
    SignalIoDevice* inDevice(){ return device_[In]; }
    int inSignalIndex(){ return signalIndex_[In]; }

    SignalIoDevice* device(int which){ return device_[which]; }
    int signalIndex(int which){ return signalIndex_[which]; }
    const std::string& bodyName(int which) { return bodyName_[which]; }
    const std::string& deviceName(int which) { return deviceName_[which]; }

private:
    SignalIoDevicePtr device_[2];
    int signalIndex_[2];
    std::string bodyName_[2];
    std::string deviceName_[2];
};
typedef ref_ptr<SignalIoConnection> SignalIoConnectionPtr;

class CNOID_EXPORT SignalIoConnectionMap : public Referenced
{
public:
    SignalIoConnectionMap();
    SignalIoConnectionMap(const SignalIoConnectionMap& org);
    SignalIoConnectionMap(const SignalIoConnectionMap& org, BodyCloneMap& bodyCloneMap, bool doConnection = false);

    typedef std::vector<SignalIoConnectionPtr> container;
    typedef container::iterator iterator;
    typedef container::const_iterator const_iterator;

    iterator begin(){ return connections_.begin(); }
    const_iterator begin() const { return connections_.begin(); }
    iterator end(){ return connections_.end(); }
    const_iterator end() const { return connections_.end(); }

    int numConnections() const { return connections_.size(); }
    SignalIoConnection* connection(int index){ return connections_[index]; }
    const SignalIoConnection* connection(int index) const { return connections_[index]; }
    void insert(int index, SignalIoConnection* connection);
    void removeConnectionsOfBody(Body* body);

private:
    std::vector<SignalIoConnectionPtr> connections_;
};

typedef ref_ptr<SignalIoConnectionMap> SignalIoConnectionMapPtr;

}

#endif
