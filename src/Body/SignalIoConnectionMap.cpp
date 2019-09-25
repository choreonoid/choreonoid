#include "SignalIoConnectionMap.h"
#include "SignalIoDevice.h"
#include "Body.h"
#include "BodyCloneMap.h"
#include <cnoid/ValueTree>
#include <fmt/format.h>
#include <algorithm>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

SignalIoConnection::SignalIoConnection()
{
    for(int i=0; i < 2; ++i){
        signalNumber_[i] = 0;
    }
}


SignalIoConnection::SignalIoConnection(SignalIoDevice* outDevice, int outNumber, SignalIoDevice* inDevice, int inNumber)
{
    device_[Out] = outDevice;
    signalNumber_[Out] = outNumber;
    device_[In] = inDevice;
    signalNumber_[In] = inNumber;
}


SignalIoConnection::SignalIoConnection(const SignalIoConnection& org)
{
    for(int i=0; i < 2; ++i){
        device_[i] = org.device_[i];
        signalNumber_[i] = org.signalNumber_[i];
        bodyName_[i] = org.bodyName_[i];
        deviceName_[i] = org.deviceName_[i];
    }
}


SignalIoConnection::SignalIoConnection(const SignalIoConnection& org, BodyCloneMap& bodyCloneMap)
{
    for(int i=0; i < 2; ++i){
        device_[i] = bodyCloneMap.getClone<SignalIoDevice>(org.device_[i]);
        signalNumber_[i] = org.signalNumber_[i];
        bodyName_[i] = org.bodyName_[i];
        deviceName_[i] = org.deviceName_[i];
    }
}


const std::string& SignalIoConnection::bodyName(int which) const
{
    if(auto device = device_[which]){
        if(auto body = device->body()){
            return body->name();
        }
    }
    return bodyName_[which];
}


const std::string& SignalIoConnection::deviceName(int which) const
{
    if(auto device = device_[which]){
        return device->name();
    } else {
        return deviceName_[which];
    }
}


void SignalIoConnection::setDevice(int which, SignalIoDevice* device)
{
    device_[which] = device;
    if(device){
        deviceName_[which] = device->name();
        if(auto body = device->body()){
            bodyName_[which] = body->name();
        }
    }
}


void SignalIoConnection::setNames(int which, const std::string& bodyName, const std::string& deviceName)
{
    bodyName_[which] = bodyName;
    deviceName_[which] = deviceName;
    device_[which].reset();
}


bool SignalIoConnection::establishConnection()
{
    if(!hasDeviceInstances()){
        connection.disconnect();
        return false;
    }

    SignalIoDevicePtr destDevice = inDevice();
    auto destNumber = inSignalNumber();
    connection.reset(
        outDevice()->sigSignalOutput(outSignalNumber()).connect(
            [destDevice, destNumber](bool on){
                destDevice->setIn(destNumber, on, true); }));
    return true;
}


void SignalIoConnection::releaseConnection()
{
    connection.disconnect();
}


bool SignalIoConnection::read(const Mapping& archive)
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


bool SignalIoConnection::write(Mapping& archive) const
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


SignalIoConnectionMap::SignalIoConnectionMap()
{

}


SignalIoConnectionMap::SignalIoConnectionMap(const SignalIoConnectionMap& org)
{
    for(auto& connection : org.connections_){
        append(new SignalIoConnection(*connection));
    }
}


SignalIoConnectionMap::SignalIoConnectionMap(const SignalIoConnectionMap& org, BodyCloneMap& bodyCloneMap)
{
    for(auto& connection : org.connections_){
        append(new SignalIoConnection(*connection, bodyCloneMap));
    }
}


SignalIoConnectionMap* SignalIoConnectionMap::clone(BodyCloneMap& bodyCloneMap) const
{
    return new SignalIoConnectionMap(*this, bodyCloneMap);
}


void SignalIoConnectionMap::insert(int index, SignalIoConnection* connection)
{
    if(index >= connections_.size()){
        connections_.push_back(connection);
    } else {
        connections_.insert(connections_.begin() + index, connection);
    }
}


void SignalIoConnectionMap::append(SignalIoConnection* connection)
{
    connections_.push_back(connection);
}


void SignalIoConnectionMap::remove(SignalIoConnection* connection)
{
    connections_.erase(
        std::remove(connections_.begin(), connections_.end(), connection),
        connections_.end());
}


void SignalIoConnectionMap::establishConnections()
{
    for(auto& connection : connections_){
        connection->establishConnection();
    }
}


void SignalIoConnectionMap::releaseConnections()
{
    for(auto& connection : connections_){
        connection->releaseConnection();
    }
}


bool SignalIoConnectionMap::read(const Mapping& archive)
{
    auto& typeNode = archive.get("type");
    if(typeNode.toString() != "SignalIoConnectionMap"){
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
            SignalIoConnectionPtr connection = new SignalIoConnection;
            if(connection->read(node)){
                append(connection);
            }
        }
    }

    return true;
}


bool SignalIoConnectionMap::write(Mapping& archive) const
{
    archive.write("type", "SignalIoConnectionMap");
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
