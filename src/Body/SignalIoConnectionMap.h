#ifndef CNOID_BODY_SIGNAL_IO_CONNECTION_MAP_H
#define CNOID_BODY_SIGNAL_IO_CONNECTION_MAP_H

#include <cnoid/Referenced>
#include <cnoid/Signal>
#include <string>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class SignalIoDevice;
typedef ref_ptr<SignalIoDevice> SignalIoDevicePtr;

class Body;
class BodyCloneMap;
class Mapping;

class CNOID_EXPORT SignalIoConnection : public Referenced
{
public:
    SignalIoConnection();
    SignalIoConnection(SignalIoDevice* outDevice, int outNumber, SignalIoDevice* inDevice, int inNumber);
    SignalIoConnection(const SignalIoConnection& org);
    SignalIoConnection(const SignalIoConnection& org, BodyCloneMap& bodyCloneMap);

    enum IoType { In = 0, Out = 1 };

    SignalIoDevice* device(int which) const { return device_[which]; }
    int signalNumber(int which) const { return signalNumber_[which]; }
    const std::string& bodyName(int which) const;
    const std::string& deviceName(int which) const;

    bool hasDeviceInstances() const {
        return (device_[In] != nullptr) && (device_[Out] != nullptr);
    }

    void setDevice(int which, SignalIoDevice* device);
    void setNames(int which, const std::string& bodyName, const std::string& deviceName);
    void setSignalNumber(int which, int number){ signalNumber_[which] = number; }

    SignalIoDevice* outDevice(){ return device(Out); }
    int outSignalNumber() const { return signalNumber(Out); }
    const std::string& outBodyName(int which) const { return bodyName(Out); }
    const std::string& outDeviceName(int which) const { return deviceName(Out); }

    SignalIoDevice* inDevice(){ return device(In); }
    int inSignalNumber() const { return signalNumber(In); }
    const std::string& inBodyName(int which) const { return bodyName(In); }
    const std::string& inDeviceName(int which) const { return deviceName(In); }
    
    void setInDevice(SignalIoDevice* device){ setDevice(In, device); }
    void setInSignalNumber(int index){ setSignalNumber(In, index); }
    void setOutDevice(SignalIoDevice* device){ setDevice(Out, device); }
    void setOutSignalNumber(int index){ setSignalNumber(Out, index); }

    bool establishConnection();
    void releaseConnection();

    bool read(const Mapping& archive);
    bool write(Mapping& archive) const;

private:
    SignalIoDevicePtr device_[2];
    int signalNumber_[2];
    std::string bodyName_[2];
    std::string deviceName_[2];
    ScopedConnection connection;
};
typedef ref_ptr<SignalIoConnection> SignalIoConnectionPtr;

class CNOID_EXPORT SignalIoConnectionMap : public Referenced
{
public:
    SignalIoConnectionMap();
    SignalIoConnectionMap(const SignalIoConnectionMap& org);
    SignalIoConnectionMap(const SignalIoConnectionMap& org, BodyCloneMap& bodyCloneMap);

    SignalIoConnectionMap* clone(BodyCloneMap& bodyCloneMap) const;

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
    void append(SignalIoConnection* connection);
    void remove(SignalIoConnection* connection);

    void establishConnections();
    void releaseConnections();

    bool read(const Mapping& archive);
    bool write(Mapping& archive) const;

private:
    std::vector<SignalIoConnectionPtr> connections_;
};

typedef ref_ptr<SignalIoConnectionMap> SignalIoConnectionMapPtr;

}

#endif
