/**
   \file
   \author Ikumi Susa
*/

#include <cnoid/Device>
#include <cnoid/YAMLBodyLoader>
#include <cnoid/YAMLReader>
#include <cnoid/AGXBodyExtension>
#include <cnoid/AGXBody>
#include <cnoid/AGXScene>
#include "../AGXConvert.h"

using namespace std;
namespace cnoid{

/////////////////////////////////////////////////////////////////////////
// AGXBreakableJointDevice
struct AGXBreakableJointDeviceDesc
{
    AGXBreakableJointDeviceDesc(){}
};

class AGXBreakableJointDevice : private AGXBreakableJointDeviceDesc, public Device
{
public:
    static bool createAGXBreakableJointDevice(YAMLBodyLoader& loader, Mapping& node);
    AGXBreakableJointDevice(const AGXBreakableJointDeviceDesc& desc, Mapping* info);
    AGXBreakableJointDevice(const AGXBreakableJointDevice& org, bool copyStateOnly = false);
    virtual const char* typeName() override;
    void copyStateFrom(const AGXBreakableJointDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual Device* clone() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    void setDesc(const AGXBreakableJointDeviceDesc& desc);
    void getDesc(AGXBreakableJointDeviceDesc& desc);
    const Mapping* info() const;
    Mapping* info();
    void resetInfo(Mapping* info);

private:
    MappingPtr m_info;
};
typedef ref_ptr<AGXBreakableJointDevice> AGXBreakableJointDevicePtr;

bool AGXBreakableJointDevice::createAGXBreakableJointDevice(YAMLBodyLoader&loader, Mapping&node)
{
    MappingPtr info = node.cloneMapping();
    AGXBreakableJointDeviceDesc desc;
    AGXBreakableJointDevicePtr jointDevice = new AGXBreakableJointDevice(desc, info);
    //node.clear();
    return loader.readDevice(jointDevice, node);
}

AGXBreakableJointDevice::AGXBreakableJointDevice(const AGXBreakableJointDeviceDesc& desc, Mapping* info) :
    AGXBreakableJointDeviceDesc(desc)
{
    resetInfo(info);
}

AGXBreakableJointDevice::AGXBreakableJointDevice(const AGXBreakableJointDevice& org, bool copyStateOnly) :
    Device(org, copyStateOnly)
{
    copyStateFrom(org);
}

const char*AGXBreakableJointDevice::typeName()
{
    return "AGXBreakableJointDevice";
}

void AGXBreakableJointDevice::copyStateFrom(const AGXBreakableJointDevice& other)
{
    AGXBreakableJointDeviceDesc desc;
    AGXBreakableJointDevice& device = const_cast<AGXBreakableJointDevice&>(other);
    device.getDesc(desc);  // Need to get desc. So do const_cast above.
    setDesc(desc);
    resetInfo(device.info());
}

void AGXBreakableJointDevice::copyStateFrom(const DeviceState&other)
{
    if(typeid(other) != typeid(AGXBreakableJointDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const AGXBreakableJointDevice&>(other));
}

DeviceState* AGXBreakableJointDevice::cloneState() const
{
    return new AGXBreakableJointDevice(*this, false);
}

Device*AGXBreakableJointDevice::clone() const
{
    return new AGXBreakableJointDevice(*this);
}

void AGXBreakableJointDevice::forEachActualType(std::function<bool(const std::type_info&type)> func)
{
    if(!func(typeid(AGXBreakableJointDevice))){
        Device::forEachActualType(func);
    }
}

int AGXBreakableJointDevice::stateSize() const
{
    return 1;
}

const double* AGXBreakableJointDevice::readState(const double* buf)
{
    return buf + 1;
}

double* AGXBreakableJointDevice::writeState(double* out_buf) const
{
    return out_buf + 1;
}

void AGXBreakableJointDevice::setDesc(const AGXBreakableJointDeviceDesc& desc)
{
    static_cast<AGXBreakableJointDeviceDesc&>(*this) = desc;
}

void AGXBreakableJointDevice::getDesc(AGXBreakableJointDeviceDesc& desc)
{
    desc = static_cast<AGXBreakableJointDeviceDesc&>(*this);
}

const Mapping* AGXBreakableJointDevice::info() const
{
    return m_info;
}

Mapping* AGXBreakableJointDevice::info()
{
    return m_info;
}

void AGXBreakableJointDevice::resetInfo(Mapping* info)
{
    m_info = info;
}

/////////////////////////////////////////////////////////////////////////
// JointBreaker
class JointBreaker : public agxSDK::StepEventListener
{
private:
    //agx::Constraint1DOFObserver m_joint;
    agx::ConstraintObserver m_joint;
    const double m_breakLimitForce;
    agx::Vec3 m_validAxis;
    const double m_period;
    bool bTimerOn;
    double startTime;

public:
    JointBreaker(agx::Constraint* joint, const double& breakLimitForce, const agx::Vec3& validAxis, const double& period) :
        m_joint(joint),
        m_breakLimitForce(breakLimitForce),
        m_validAxis(validAxis),
        m_period(period)
    {
        m_validAxis.normalize();
        m_joint->setEnableComputeForces(true);
        bTimerOn = false;
        startTime = 0.0;
    }

   virtual void post( const agx::TimeStamp& t )
    {
        agx::Vec3 force;
        agx::Vec3 torque;
        m_joint->getLastForce((agx::UInt)0, force, torque); // world coord.
        for(int i = 0; i < 3; i++){
            force[i] = force[i] * m_validAxis[i];
        }

        agx::Real duration = 0.0;
        if(force.length() >= m_breakLimitForce){
            if(!bTimerOn){
                bTimerOn = true;
                startTime = t;
            }
            duration = t - startTime;
            if(duration > m_period){
                m_joint->setEnable(false);
            }
        }else{
            bTimerOn = false;
        }
        std::cout << "AGXBreakableJoint " << force.length() << " " << m_breakLimitForce << std::endl;
    }
};

/////////////////////////////////////////////////////////////////////////
// AGXBreakableJoint
class AGXBreakableJoint : public AGXBodyExtension
{
public:
    static bool createAGXBreakableJoint(cnoid::AGXBody* agxBody);
    AGXBreakableJoint(AGXBreakableJointDevice* device, AGXBody* agxBody);
private:
    AGXBreakableJointDevicePtr m_device;
};
typedef ref_ptr<AGXBreakableJoint> AGXBreakableJointPtr;


bool AGXBreakableJoint::createAGXBreakableJoint(cnoid::AGXBody* agxBody)
{
    DeviceList<> devices = agxBody->body()->devices();
    DeviceList<AGXBreakableJointDevice> jointDevices;
    jointDevices.extractFrom(devices);
    for(auto device : jointDevices){
        agxBody->addAGXBodyExtension(new cnoid::AGXBreakableJoint(device, agxBody));
    }
    return true;
}

AGXBreakableJoint::AGXBreakableJoint(AGXBreakableJointDevice* device, AGXBody* agxBody) :
    AGXBodyExtension(agxBody),
    m_device(device)
{
    struct AGXBreakableJointParam
    {
        AGXBreakableJointParam(){
            breakLimitForce = std::numeric_limits<double>::max();
            period = 0.0;
            jointType = "fixed";
            position = Vector3();
            jointAxis = Vector3(0, 0, 1);
            validAxis = Vector3(1, 1, 1);
            jointRange[0] = -std::numeric_limits<double>::max();
            jointRange[1] = std::numeric_limits<double>::max();
        }
        double breakLimitForce, period;
        string jointType;
        string  link1Name, link2Name;
        Vector3 position;   // joint position on link1 coordinate
        Vector3 jointAxis;  // joint axis on link1 coordinate
        Vector3 validAxis;
        Vector2 jointRange;
    }jp;

    // Get parameters from yaml
    if(!m_device) return;
    Mapping& jointDeviceInfo = *m_device->info();
    jointDeviceInfo.read("breakLimitForce", jp.breakLimitForce);
    jointDeviceInfo.read("period", jp.period);
    jointDeviceInfo.read("jointType", jp.jointType);
    jointDeviceInfo.read("link1Name", jp.link1Name);
    jointDeviceInfo.read("link2Name", jp.link2Name);
    agxConvert::setVector(jointDeviceInfo.find("position"), jp.position);
    agxConvert::setVector(jointDeviceInfo.find("jointAxis"), jp.jointAxis);
    agxConvert::setVector(jointDeviceInfo.find("validAxis"), jp.validAxis);
    agxConvert::setVector(jointDeviceInfo.find("jointRange"), jp.jointRange);

    AGXElementaryConstraint base, range, lock;
    jointDeviceInfo.read("jointCompliance", base.compliance);
    jointDeviceInfo.read("jointDamping", base.damping);
    jointDeviceInfo.read("jointRangeCompliance", range.compliance);
    jointDeviceInfo.read("jointRangeDamping", range.damping);
    jointDeviceInfo.read("jointLock", lock.enable);
    jointDeviceInfo.read("jointLockCompliance", lock.compliance);
    jointDeviceInfo.read("jointLockDamping", lock.damping);
    Vector2 baseForceRange, rangeForceRange, lockForceRange;
    if(agxConvert::setVector(jointDeviceInfo.find("jointForceRange"), baseForceRange)){
        base.forceRange = agx::RangeReal(baseForceRange(0), baseForceRange(1));
    }
    if(agxConvert::setVector(jointDeviceInfo.find("jointRangeForceRange"), rangeForceRange)){
        range.forceRange = agx::RangeReal(rangeForceRange(0), rangeForceRange(1));
    }
    if(agxConvert::setVector(jointDeviceInfo.find("jointLockForceRange"), rangeForceRange)){
        lock.forceRange = agx::RangeReal(lockForceRange(0), lockForceRange(1));
    }

    // Create breakable joint
    AGXLink* const agxLink1 = getAGXBody()->getAGXLink(jp.link1Name);
    if(!agxLink1) return;
    Link* const link1 = agxLink1->getOrgLink();
    const Vector3 p = link1->attitude() * jp.position + link1->p();
    const Vector3 a = link1->attitude() * jp.jointAxis;
    const agx::Vec3 validAxis = agxConvert::toAGX(link1->attitude() * jp.validAxis);

    auto createConstraint = [&](AGXConstraintDesc& jd){
        jd.rigidBodyA = getAGXBody()->getAGXRigidBody(jp.link1Name);
        jd.rigidBodyB = getAGXBody()->getAGXRigidBody(jp.link2Name);
        jd.compliance = base.compliance;
        jd.damping = base.damping;
        jd.forceRange = base.forceRange;
        return AGXObjectFactory::createConstraint(jd);
    };

    agx::ConstraintRef joint;
    if(jp.jointType == "revolute"){
        AGXHingeDesc hd;
        hd.frameAxis = agxConvert::toAGX(a);
        hd.frameCenter = agxConvert::toAGX(p);
        hd.range.enable = true;
        hd.range.range = agx::RangeReal(agx::degreesToRadians(jp.jointRange[0]), agx::degreesToRadians(jp.jointRange[1]));
        hd.range.compliance = range.compliance;
        hd.range.damping = range.damping;
        hd.range.forceRange = range.forceRange;
        hd.lock.enable = lock.enable;
        hd.lock.compliance = lock.compliance;
        hd.lock.damping - lock.damping;
        hd.lock.forceRange = lock.forceRange;
        joint = createConstraint(hd);
    }else if(jp.jointType == "prismatic"){
        AGXPrismaticDesc pd;
        pd.frameAxis = agxConvert::toAGX(a);
        pd.framePoint = agxConvert::toAGX(p);
        pd.range.enable = true;
        pd.range.range = agx::RangeReal(jp.jointRange[0], jp.jointRange[1]);
        pd.range.compliance = range.compliance;
        pd.range.damping = range.damping;
        pd.range.forceRange = range.forceRange;
        pd.lock.enable = lock.enable;
        pd.lock.compliance = lock.compliance;
        pd.lock.damping - lock.damping;
        pd.lock.forceRange = lock.forceRange;
        joint = createConstraint(pd);
    }else if(jp.jointType == "fixed"){
        AGXLockJointDesc ld;
        joint = createConstraint(ld);
    }else{
        return;
    }

    getAGXBody()->getAGXScene()->getSimulation()->add(joint);
    getAGXBody()->getAGXScene()->getSimulation()
        ->add(new JointBreaker(joint, jp.breakLimitForce, validAxis, jp.period));
}

} // cnoid

namespace{
using namespace cnoid;

/////////////////////////////////////////////////////////////////////////
// Register AGXBreakableJointDevice
struct AGXBreakableJointDeviceRegistration
{
    AGXBreakableJointDeviceRegistration()
    {
        YAMLBodyLoader::addNodeType("AGXBreakableJointDevice", AGXBreakableJointDevice::createAGXBreakableJointDevice);
    }
}registrationAGXBreakableJointDevice;

/////////////////////////////////////////////////////////////////////////
// Register AGXBreakableJoint
struct AGXBreakableJointRegistration
{
    AGXBreakableJointRegistration() {
        AGXBody::addAGXBodyExtensionAdditionalFunc("AGXBreakableJoint", AGXBreakableJoint::createAGXBreakableJoint);
    }
}registrationAGXBreakableJoint;

}