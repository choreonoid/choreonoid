#include "IoConnectionMap.h"
#include "DigitalIoDevice.h"
#include "Body.h"
#include "BodyCloneMap.h"
#include <cnoid/ValueTree>
#include <fmt/format.h>
#include <algorithm>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

DigitalIoConnection::DigitalIoConnection()
{
    for(int i=0; i < 2; ++i){
        signalNumber_[i] = 0;
    }
}


DigitalIoConnection::DigitalIoConnection(DigitalIoDevice* outDevice, int outNumber, DigitalIoDevice* inDevice, int inNumber)
{
    device_[Out] = outDevice;
    signalNumber_[Out] = outNumber;
    device_[In] = inDevice;
    signalNumber_[In] = inNumber;
}


DigitalIoConnection::DigitalIoConnection(const DigitalIoConnection& org)
{
    for(int i=0; i < 2; ++i){
        device_[i] = org.device_[i];
        signalNumber_[i] = org.signalNumber_[i];
        bodyName_[i] = org.bodyName_[i];
        deviceName_[i] = org.deviceName_[i];
    }
}


DigitalIoConnection::DigitalIoConnection(const DigitalIoConnection& org, BodyCloneMap& bodyCloneMap)
{
    for(int i=0; i < 2; ++i){
        device_[i] = bodyCloneMap.getClone<DigitalIoDevice>(org.device_[i]);
        signalNumber_[i] = org.signalNumber_[i];
        bodyName_[i] = org.bodyName_[i];
        deviceName_[i] = org.deviceName_[i];
    }
}


const std::string& DigitalIoConnection::bodyName(int which) const
{
    if(auto device = device_[which]){
        if(auto body = device->body()){
            return body->name();
        }
    }
    return bodyName_[which];
}


const std::string& DigitalIoConnection::deviceName(int which) const
{
    if(auto device = device_[which]){
        return device->name();
    } else {
        return deviceName_[which];
    }
}


void DigitalIoConnection::setDevice(int which, DigitalIoDevice* device)
{
    device_[which] = device;
    if(device){
        deviceName_[which] = device->name();
        if(auto body = device->body()){
            bodyName_[which] = body->name();
        }
    }
}


void DigitalIoConnection::setNames(int which, const std::string& bodyName, const std::string& deviceName)
{
    bodyName_[which] = bodyName;
    deviceName_[which] = deviceName;
    device_[which].reset();
}


bool DigitalIoConnection::establishConnection()
{
    if(!hasDeviceInstances()){
        connection.disconnect();
        return false;
    }

    DigitalIoDevicePtr destDevice = inDevice();
    auto destNumber = inSignalNumber();
    connection.reset(
        outDevice()->sigSignalOutput(outSignalNumber()).connect(
            [destDevice, destNumber](bool on){
                destDevice->setIn(destNumber, on, true); }));
    return true;
}


void DigitalIoConnection::releaseConnection()
{
    connection.disconnect();
}


bool DigitalIoConnection::read(const Mapping& archive)
{
    device_[Out] = nullptr;
    device_[In] = nullptr;

    bodyName_[Out] = archive.get<string>("outBody");
    bodyName_[In] = archive.get<string>("inBody");
    
    deviceName_[Out] = archive.get("outDevice", "");
    deviceName_[In] = archive.get("inDevice", "");

    signalNumber_[Out] = archive.get<int>("outSignalNumber");
    signalNumber_[In] = archive.get<int>("inSignalNumber");

    return true;
}


bool DigitalIoConnection::write(Mapping& archive) const
{
    archive.write("outBody", bodyName(Out), DOUBLE_QUOTED);
    auto& outDeviceName = deviceName(Out);
    if(!outDeviceName.empty()){
        archive.write("outDevice", outDeviceName, DOUBLE_QUOTED);
    }
    archive.write("outSignalNumber", signalNumber(Out));

    archive.write("inBody", bodyName(In), DOUBLE_QUOTED);
    auto& inDeviceName = deviceName(In);
    if(!inDeviceName.empty()){
        archive.write("inDevice", inDeviceName, DOUBLE_QUOTED);
    }
    archive.write("inSignalNumber", signalNumber(In));

    return true;
}


IoConnectionMap::IoConnectionMap()
{

}


IoConnectionMap::IoConnectionMap(const IoConnectionMap& org)
{
    for(auto& connection : org.connections_){
        append(new DigitalIoConnection(*connection));
    }
}


IoConnectionMap::IoConnectionMap(const IoConnectionMap& org, BodyCloneMap& bodyCloneMap)
{
    for(auto& connection : org.connections_){
        append(new DigitalIoConnection(*connection, bodyCloneMap));
    }
}


IoConnectionMap* IoConnectionMap::clone(BodyCloneMap& bodyCloneMap) const
{
    return new IoConnectionMap(*this, bodyCloneMap);
}


void IoConnectionMap::insert(int index, DigitalIoConnection* connection)
{
    if(index >= connections_.size()){
        connections_.push_back(connection);
    } else {
        connections_.insert(connections_.begin() + index, connection);
    }
}


void IoConnectionMap::append(DigitalIoConnection* connection)
{
    connections_.push_back(connection);
}


void IoConnectionMap::remove(DigitalIoConnection* connection)
{
    connections_.erase(
        std::remove(connections_.begin(), connections_.end(), connection),
        connections_.end());
}


void IoConnectionMap::establishConnections()
{
    for(auto& connection : connections_){
        connection->establishConnection();
    }
}


void IoConnectionMap::releaseConnections()
{
    for(auto& connection : connections_){
        connection->releaseConnection();
    }
}


bool IoConnectionMap::read(const Mapping& archive)
{
    auto& typeNode = archive.get("type");
    if(typeNode.toString() != "IoConnectionMap"){
        typeNode.throwException(
            format(_("{0} cannot be loaded as a signal I/O connection map"), typeNode.toString()));
    }
        
    auto& versionNode = archive.get("formatVersion");
    auto version = versionNode.toDouble();
    if(version != 1.0){
        versionNode.throwException(format(_("Format version {0} is not supported."), version));
    }

    auto& connectionNodes = *archive.findListing("connections");
    if(connectionNodes.isValid()){
        for(int i=0; i < connectionNodes.size(); ++i){
            auto& node = *connectionNodes[i].toMapping();
            DigitalIoConnectionPtr connection = new DigitalIoConnection;
            if(connection->read(node)){
                append(connection);
            }
        }
    }

    return true;
}


bool IoConnectionMap::write(Mapping& archive) const
{
    archive.write("type", "IoConnectionMap");
    archive.write("formatVersion", 1.0);

    if(!connections_.empty()){
        Listing& connectionNodes = *archive.createListing("connections");
        for(auto& connection : connections_){
            MappingPtr node = new Mapping;
            if(connection->write(*node)){
                connectionNodes.append(node);
            }
        }
    }
    return true;
}
