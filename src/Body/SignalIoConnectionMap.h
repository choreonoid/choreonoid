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

    enum IoType { In = 0, Out = 1 };

    SignalIoDevice* inDevice(){ return device_[In]; }
    int inSignalIndex() const { return signalIndex_[In]; }
    SignalIoDevice* outDevice(){ return device_[Out]; }
    int outSignalIndex() const { return signalIndex_[Out]; }

    void setInDevice(SignalIoDevice* device){ setDevice(In, device); }
    void setInSignalIndex(int index){ signalIndex_[In] = index; }
    void setOutDevice(SignalIoDevice* device){ setDevice(Out, device); }
    void setOutSignalIndex(int index){ signalIndex_[Out] = index; }

    SignalIoDevice* device(IoType which) const { return device_[which]; }
    int signalIndex(IoType which) const { return signalIndex_[which]; }
    const std::string& bodyName(IoType which) const;
    const std::string& deviceName(IoType which) const;

    void setDevice(IoType which, SignalIoDevice* device);
    void setNames(IoType which, const std::string& bodyName, const std::string& deviceName);

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
