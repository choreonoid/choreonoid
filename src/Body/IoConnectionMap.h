#ifndef CNOID_BODY_IO_CONNECTION_MAP_H
#define CNOID_BODY_IO_CONNECTION_MAP_H

#include <cnoid/CloneableReferenced>
#include <cnoid/Signal>
#include <string>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class DigitalIoDevice;
typedef ref_ptr<DigitalIoDevice> DigitalIoDevicePtr;

class Body;
class Mapping;

class CNOID_EXPORT DigitalIoConnection : public CloneableReferenced
{
public:
    DigitalIoConnection();
    DigitalIoConnection(DigitalIoDevice* outDevice, int outIndex, DigitalIoDevice* inDevice, int inIndex);

    DigitalIoConnection* clone() const {
        return static_cast<DigitalIoConnection*>(doClone(nullptr));
    }
    DigitalIoConnection* clone(CloneMap& cloneMap) const {
        return static_cast<DigitalIoConnection*>(doClone(&cloneMap));
    }

    enum IoType { In = 0, Out = 1 };

    DigitalIoDevice* device(int which) const { return device_[which]; }
    int signalIndex(int which) const { return signalIndex_[which]; }
    const std::string& bodyName(int which) const;
    const std::string& deviceName(int which) const;

    bool hasDeviceInstances() const {
        return (device_[In] != nullptr) && (device_[Out] != nullptr);
    }

    void setDevice(int which, DigitalIoDevice* device);
    void setNames(int which, const std::string& bodyName, const std::string& deviceName);
    void setSignalIndex(int which, int index){ signalIndex_[which] = index; }

    DigitalIoDevice* outDevice(){ return device(Out); }
    int outSignalIndex() const { return signalIndex(Out); }
    const std::string& outBodyName(int which) const { return bodyName(Out); }
    const std::string& outDeviceName(int which) const { return deviceName(Out); }

    DigitalIoDevice* inDevice(){ return device(In); }
    int inSignalIndex() const { return signalIndex(In); }
    const std::string& inBodyName(int which) const { return bodyName(In); }
    const std::string& inDeviceName(int which) const { return deviceName(In); }
    
    void setInDevice(DigitalIoDevice* device){ setDevice(In, device); }
    void setInSignalIndex(int index){ setSignalIndex(In, index); }
    void setOutDevice(DigitalIoDevice* device){ setDevice(Out, device); }
    void setOutSignalIndex(int index){ setSignalIndex(Out, index); }

    bool establishConnection();
    void releaseConnection();

    bool read(const Mapping& archive);
    bool write(Mapping& archive) const;

protected:
    DigitalIoConnection(const DigitalIoConnection& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    DigitalIoDevicePtr device_[2];
    int signalIndex_[2];
    std::string bodyName_[2];
    std::string deviceName_[2];
    ScopedConnection connection;
};
typedef ref_ptr<DigitalIoConnection> DigitalIoConnectionPtr;


class CNOID_EXPORT IoConnectionMap : public CloneableReferenced
{
public:
    IoConnectionMap();

    IoConnectionMap* clone() const {
        return static_cast<IoConnectionMap*>(doClone(nullptr));
    }
    IoConnectionMap* clone(CloneMap& cloneMap) const {
        return static_cast<IoConnectionMap*>(doClone(&cloneMap));
    }

    typedef std::vector<DigitalIoConnectionPtr> container;
    typedef container::iterator iterator;
    typedef container::const_iterator const_iterator;

    iterator begin(){ return connections_.begin(); }
    const_iterator begin() const { return connections_.begin(); }
    iterator end(){ return connections_.end(); }
    const_iterator end() const { return connections_.end(); }

    int numConnections() const { return connections_.size(); }
    DigitalIoConnection* connection(int index){ return connections_[index]; }
    const DigitalIoConnection* connection(int index) const { return connections_[index]; }
    void insert(int index, DigitalIoConnection* connection);
    void append(DigitalIoConnection* connection);
    void remove(DigitalIoConnection* connection);

    void establishConnections();
    void releaseConnections();

    bool read(const Mapping& archive);
    bool write(Mapping& archive) const;

protected:
    IoConnectionMap(const IoConnectionMap& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    std::vector<DigitalIoConnectionPtr> connections_;
};

typedef ref_ptr<IoConnectionMap> IoConnectionMapPtr;

}

#endif
