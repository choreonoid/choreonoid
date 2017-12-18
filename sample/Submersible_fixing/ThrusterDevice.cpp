/*
 * ThrusterDevice.cpp
 *
 *  Created on: 2017/10/16
 *      Author: anazawa
 */
#include "ThrusterDevice.h"
#include <cnoid/YAMLBodyLoader>
#include <cnoid/YAMLReader>
//#include <boost/bind.hpp>
//#include <cnoid/Link>
//#include <cnoid/SimulatorItem>
//#include <cnoid/SubSimulatorItem>
#include <cnoid/MessageView>
//#include <cnoid/WorldItem>
//#include <cnoid/ItemManager>
//#include <cnoid/Archive>
//#include <cnoid/Body>
//#include <cnoid/Light>
//#include <cnoid/Joystick>
//#include <cnoid/EigenTypes>
//#include <cnoid/ControllerItem>

using namespace std;
using namespace cnoid;
using boost::format;

//namespace cnoid {

bool readThrusterDevice(YAMLBodyLoader& loader, Mapping& node)
{
	ThrusterDevicePtr thruster = new ThrusterDevice;
	return loader.readDevice(thruster, node);
}

struct TypeRegistration
{
	TypeRegistration() {
		YAMLBodyLoader::addNodeType("ThrusterDevice", readThrusterDevice);
	}
} registration;

ThrusterDevice::ThrusterDevice()
{
	initialize();
}

ThrusterDevice::ThrusterDevice(const ThrusterDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
	copyStateFrom(org);
}

const char* ThrusterDevice::typeName()
{
	return "ThrusterDevice";
}

void ThrusterDevice::copyStateFrom(const ThrusterDevice& other)
{
	hydraulic_ = other.hydraulic_;
}

void ThrusterDevice::copyStateFrom(const DeviceState& other)
{
	if(typeid(other) != typeid(ThrusterDevice)){
		throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");

	}
	copyStateFrom(static_cast<const ThrusterDevice&>(other));

}

DeviceState* ThrusterDevice::cloneState() const
{
	return new ThrusterDevice(*this, false);
}

Device* ThrusterDevice::clone() const
{
	return new ThrusterDevice(*this);
}

void ThrusterDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
	if(!func(typeid(ThrusterDevice))){
		Device::forEachActualType(func);

	}
}

int ThrusterDevice::stateSize() const
{
	return 1;
}

const double* ThrusterDevice::readState(const double* buf)
{
	hydraulic_ = buf[0];
	return buf + 1;
}

double* ThrusterDevice::writeState(double* out_buf) const
{
	out_buf[0] = hydraulic_;
	return out_buf + 1;
}

void ThrusterDevice::initialize()
{
	hydraulic_ = 0.0;

}

//}
