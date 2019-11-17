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
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;
    virtual bool on() const override;
    virtual void on(bool on) override;

    void setDesc(const AGXBreakableJointDeviceDesc& desc);
    void getDesc(AGXBreakableJointDeviceDesc& desc);
    const Mapping* info() const;
    Mapping* info();
    void resetInfo(Mapping* info);

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    MappingPtr m_info;
    bool on_;
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
    on_ = true;
}

AGXBreakableJointDevice::AGXBreakableJointDevice(const AGXBreakableJointDevice& org, bool copyStateOnly) :
    Device(org, copyStateOnly)
{
    copyStateFrom(org);
    on_ = org.on_;
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
    on_ = other.on_;
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

Referenced* AGXBreakableJointDevice::doClone(CloneMap*) const
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

bool AGXBreakableJointDevice::on() const
{
    return on_;
}

void AGXBreakableJointDevice::on(bool on)
{
    on_ = on;
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
struct JointBreakerDesc{

    enum BreakType { BREAK_TYPE_NONE, BREAK_TYPE_FORCE, BREAK_TYPE_IMPULSE };
    
    JointBreakerDesc(){
        joint = nullptr;
        breakType = BREAK_TYPE_FORCE;
        breakLimitForce = std::numeric_limits<double>::max();
        period = 0.0;
        breakLimitImpulse = std::numeric_limits<double>::max();
        offsetForce = 0.0;
        validAxis = agx::Vec3(1, 1, 1);
        signedAxis = agx::Vec3(0, 0, 0);
    }
    agx::ConstraintRef joint;

    BreakType breakType;
    double breakLimitForce;
    double period;
    double breakLimitImpulse;
    double offsetForce;
    agx::Vec3 validAxis;
    agx::Vec3 signedAxis;
};

class JointBreaker : public JointBreakerDesc, public agxSDK::StepEventListener
{
private:
    bool   m_bTimerOn;
    double m_startTime;
    agx::Real m_recivedImpulse;
    Device* device;

public:
    JointBreaker(const JointBreakerDesc& desc, Device* device) :
        JointBreakerDesc(desc),
        device(device)
    {
        init();
    }

    void init(){
        validAxis = validAxis.normal();
        joint->setEnableComputeForces(true);
        m_bTimerOn = false;
        m_startTime = 0.0;
        m_recivedImpulse = 0.0;
    }

    virtual void post( const agx::TimeStamp& t )
    {
        if(!joint->getEnable()) return;

        switch(breakType){
        case BREAK_TYPE_FORCE:
            breakOnForce(t);
            break;
        case BREAK_TYPE_IMPULSE:
            breakOnImpulse();
            break;
        default:
            break;
        }
        if(!device->on()){
            joint->setEnable(false);
        }
    }

    agx::Real getForce(){
        agx::Vec3 vf, vt;
        joint->getLastForce((agx::UInt)0, vf, vt); // world coord.
        for(int i = 0; i < 3; i++){
            // Filter force when sign are different
            if(signedAxis[i] != 0.0){
                if(signbit(vf[i]) != signbit(signedAxis[i])){
                    vf[i] = 0.0;
                }
            }
            vf[i] = vf[i] * validAxis[i];
        }
        LOGGER_INFO() << "AGXBreakableJoint force vector " << vf << LOGGER_ENDL();

        return std::max(0.0,  vf.length() - offsetForce);
    }

    void breakOnForce(const agx::TimeStamp& t){
        agx::Real force = getForce();
        agx::Real duration = 0.0;
        if(breakLimitForce <= force){
            if(!m_bTimerOn){
                m_bTimerOn = true;
                m_startTime = t;
            }
            duration = t - m_startTime;
            if(duration > period){
                joint->setEnable(false);
            }
        }else{
            m_bTimerOn = false;
        }
        LOGGER_INFO() << "AGXBreakableJoint force " << breakLimitForce << " " << force << LOGGER_ENDL();
    }

    void breakOnImpulse(){
        agx::Real force = getForce();
        const agx::Real& dt = getSimulation()->getTimeStep();
        m_recivedImpulse += force * dt;
        if(breakLimitImpulse <= m_recivedImpulse){
            joint->setEnable(false);
        }
        LOGGER_INFO() << "AGXBreakableJoint impulse " << force << " " << breakLimitImpulse << " " << m_recivedImpulse << LOGGER_ENDL();
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
    struct AGXBreakableJointParam : public  JointBreakerDesc
    {
        AGXBreakableJointParam(){
            jointType = "fixed";
            position = Vector3();
            jointAxis = Vector3(0, 0, 1);
            c_validAxis = agxConvert::toCnoid(validAxis);
            c_signedAxis = agxConvert::toCnoid(signedAxis);
            jointRange[0] = -std::numeric_limits<double>::max();
            jointRange[1] = std::numeric_limits<double>::max();
        }
        string jointType;
        string  link1Name, link2Name;
        Vector3 position;   // joint position on link1 coordinate
        Vector3 jointAxis;  // joint axis on link1 coordinate
        Vector3 c_validAxis;
        Vector3 c_signedAxis;
        Vector2 jointRange;
        JointBreakerDesc& getJointBreakerDesc(){
            return static_cast<JointBreakerDesc&>(*this);
        }
    }jp;

    // Get parameters from yaml
    if(!m_device) return;
    Mapping& jointDeviceInfo = *m_device->info();
    string breakType;
    if(jointDeviceInfo.read("breakType", breakType)){
        if(breakType == "force"){
            jp.breakType = JointBreakerDesc::BREAK_TYPE_FORCE;
        } else if(breakType == "impulse"){
            jp.breakType = JointBreakerDesc::BREAK_TYPE_IMPULSE;
        } else {
            jp.breakType = JointBreakerDesc::BREAK_TYPE_NONE;
        }
    }
    jointDeviceInfo.read("breakLimitForce", jp.breakLimitForce);
    jointDeviceInfo.read("breakLimitImpulse", jp.breakLimitImpulse);
    jointDeviceInfo.read("offsetForce", jp.offsetForce);
    jointDeviceInfo.read("period", jp.period);
    jointDeviceInfo.read("jointType", jp.jointType);
    jointDeviceInfo.read("link1Name", jp.link1Name);
    jointDeviceInfo.read("link2Name", jp.link2Name);
    agxConvert::setVector(jointDeviceInfo.find("position"), jp.position);
    agxConvert::setVector(jointDeviceInfo.find("jointAxis"), jp.jointAxis);
    agxConvert::setVector(jointDeviceInfo.find("validAxis"), jp.c_validAxis);
    agxConvert::setVector(jointDeviceInfo.find("signedAxis"), jp.c_signedAxis);
    agxConvert::setVector(jointDeviceInfo.find("jointRange"), jp.jointRange);

    AGXElementaryConstraint base, range, lock;
    jointDeviceInfo.read("jointCompliance", base.compliance);
    jointDeviceInfo.read("jointSpookDamping", base.spookDamping);
    jointDeviceInfo.read("jointRangeCompliance", range.compliance);
    jointDeviceInfo.read("jointRangeSpookDamping", range.spookDamping);
    jointDeviceInfo.read("jointLock", lock.enable);
    jointDeviceInfo.read("jointLockCompliance", lock.compliance);
    jointDeviceInfo.read("jointLockSpookDamping", lock.spookDamping);
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
    jp.validAxis = agxConvert::toAGX(link1->attitude() * jp.c_validAxis);
    jp.signedAxis = agxConvert::toAGX(jp.c_signedAxis);

    auto createConstraint = [&](AGXConstraintDesc& jd){
        jd.set(base);
        jd.rigidBodyA = getAGXBody()->getAGXRigidBody(jp.link1Name);
        jd.rigidBodyB = getAGXBody()->getAGXRigidBody(jp.link2Name);
        return AGXObjectFactory::createConstraint(jd);
    };

    agx::ConstraintRef joint;
    if(jp.jointType == "revolute"){
        AGXHingeDesc hd;
        hd.frameAxis = agxConvert::toAGX(a);
        hd.frameCenter = agxConvert::toAGX(p);
        hd.range.set(range);
        hd.range.enable = true;
        hd.range.range = agx::RangeReal(agx::degreesToRadians(jp.jointRange[0]), agx::degreesToRadians(jp.jointRange[1]));
        hd.lock.set(lock);
        joint = createConstraint(hd);
    }else if(jp.jointType == "prismatic"){
        AGXPrismaticDesc pd;
        pd.frameAxis = agxConvert::toAGX(a);
        pd.framePoint = agxConvert::toAGX(p);
        pd.range.set(range);
        pd.range.enable = true;
        pd.range.range = agx::RangeReal(jp.jointRange[0], jp.jointRange[1]);
        pd.lock.set(lock);
        joint = createConstraint(pd);
    }else if(jp.jointType == "fixed"){
        AGXLockJointDesc ld;
        joint = createConstraint(ld);
    }else{
        return;
    }

    agxSDK::Simulation* sim = getAGXBody()->getAGXScene()->getSimulation();
    sim->add(joint);
    jp.joint = joint;
    sim->add(new JointBreaker(jp.getJointBreakerDesc(), device));
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
