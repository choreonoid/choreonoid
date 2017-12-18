/*
 * ThrusterDevice.h
 *
 *  Created on: 2017/10/16
 *      Author: anazawa
 */

#ifndef SAMPLE_SUBMERSIBLE_FIXING_THRUSTERDEVICE_H_
#define SAMPLE_SUBMERSIBLE_FIXING_THRUSTERDEVICE_H_

#include <cnoid/Device>
#include "exportdecl.h"
#include <typeinfo>
#include <cnoid/YAMLBodyLoader>
//#include <cnoid/SimulatorItem>
//#include <cnoid/SubSimulatorItem>
#include <cnoid/Item>

using namespace std;

namespace cnoid {

//class SimulationBodyImpl;

class CNOID_EXPORT ThrusterDevice : public Device
{
public:
//	static void initializeClass(ExtensionManager* ext);
//	static bool readThrusterDevice(YAMLBodyLoader& loader, Mapping& node);

	ThrusterDevice();
	ThrusterDevice(const ThrusterDevice& org, bool copyStateOnly = false);
	virtual const char* typeName() override;
	void copyStateFrom(const ThrusterDevice& other);
	virtual void copyStateFrom(const DeviceState& other) override;
	virtual DeviceState* cloneState() const override;
	virtual Device* clone() const override;
	virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
	virtual int stateSize() const override;
	virtual const double* readState(const double* buf) override;
	virtual double* writeState(double* out_buf) const override;

	void initialize();
	const double hydraulic() const { return hydraulic_; }
	void setHydraulic(const double& h) { hydraulic_ = h; }

//protected:
//    virtual void doPutProperties(PutPropertyFunction& putProperty);
//    virtual bool store(Archive& archive);
//    virtual bool restore(const Archive& archive);

private:
	double hydraulic_;
//	SimulatorItem* simulatorItem;
//	Body* simBody;
//	SimulatorItemImpl* impl;

//	void applyResistanceForce();
};

typedef ref_ptr<ThrusterDevice> ThrusterDevicePtr;

}

#endif /* SAMPLE_SUBMERSIBLE_FIXING_THRUSTERDEVICE_H_ */
