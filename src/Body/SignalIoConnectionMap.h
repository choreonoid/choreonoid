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

    SignalIoDevice* outDevice(){ return outDevice_; }
    int outSignalIndex(){ return outIndex_; }
    SignalIoDevice* inDevice(){ return inDevice_; }
    int inSignalIndex(){ return inIndex_; }

    enum { OUT = 0, IN = 1 };
    SignalIoDevice* device(int which){ return (which == 0) ? outDevice_ : inDevice_; }
    int signalIndex(int which){ return (which == 0) ? outIndex_ : inIndex_; }

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

    void appendConnection(SignalIoConnection* connection);
    
    void removeConnectionsOfBody(Body* body);

private:
    std::vector<SignalIoConnectionPtr> connections_;
};

typedef ref_ptr<SignalIoConnectionMap> SignalIoConnectionMapPtr;

}

#endif
