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
// AGXMagneticJointDevice
struct AGXMagneticJointDeviceDesc
{
    AGXMagneticJointDeviceDesc(){}
};

class AGXMagneticJointDevice : private AGXMagneticJointDeviceDesc, public Device
{
public:
    static bool createAGXMagneticJointDevice(YAMLBodyLoader& loader, Mapping& node);
    AGXMagneticJointDevice(const AGXMagneticJointDeviceDesc& desc, Mapping* info);
    AGXMagneticJointDevice(const AGXMagneticJointDevice& org, bool copyStateOnly = false);
    virtual const char* typeName() override;
    void copyStateFrom(const AGXMagneticJointDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    void setDesc(const AGXMagneticJointDeviceDesc& desc);
    void getDesc(AGXMagneticJointDeviceDesc& desc);
    const Mapping* info() const;
    Mapping* info();
    void resetInfo(Mapping* info);

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    MappingPtr m_info;
};
typedef ref_ptr<AGXMagneticJointDevice> AGXMagneticJointDevicePtr;

bool AGXMagneticJointDevice::createAGXMagneticJointDevice(YAMLBodyLoader&loader, Mapping&node)
{
    MappingPtr info = node.cloneMapping();
    AGXMagneticJointDeviceDesc desc;
    AGXMagneticJointDevicePtr jointDevice = new AGXMagneticJointDevice(desc, info);
    //node.clear();
    return loader.readDevice(jointDevice, node);
}

AGXMagneticJointDevice::AGXMagneticJointDevice(const AGXMagneticJointDeviceDesc& desc, Mapping* info) :
    AGXMagneticJointDeviceDesc(desc)
{
    resetInfo(info);
}

AGXMagneticJointDevice::AGXMagneticJointDevice(const AGXMagneticJointDevice& org, bool copyStateOnly) :
    Device(org, copyStateOnly)
{
    copyStateFrom(org);
}

const char*AGXMagneticJointDevice::typeName()
{
    return "AGXMagneticJointDevice";
}

void AGXMagneticJointDevice::copyStateFrom(const AGXMagneticJointDevice& other)
{
    AGXMagneticJointDeviceDesc desc;
    AGXMagneticJointDevice& device = const_cast<AGXMagneticJointDevice&>(other);
    device.getDesc(desc);  // Need to get desc. So do const_cast above.
    setDesc(desc);
    resetInfo(device.info());
}

void AGXMagneticJointDevice::copyStateFrom(const DeviceState&other)
{
    if(typeid(other) != typeid(AGXMagneticJointDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const AGXMagneticJointDevice&>(other));
}

DeviceState* AGXMagneticJointDevice::cloneState() const
{
    return new AGXMagneticJointDevice(*this, false);
}

Referenced* AGXMagneticJointDevice::doClone(CloneMap*) const
{
    return new AGXMagneticJointDevice(*this);
}

void AGXMagneticJointDevice::forEachActualType(std::function<bool(const std::type_info&type)> func)
{
    if(!func(typeid(AGXMagneticJointDevice))){
        Device::forEachActualType(func);
    }
}

int AGXMagneticJointDevice::stateSize() const
{
    return 1;
}

const double* AGXMagneticJointDevice::readState(const double* buf)
{
    return buf + 1;
}

double* AGXMagneticJointDevice::writeState(double* out_buf) const
{
    return out_buf + 1;
}

void AGXMagneticJointDevice::setDesc(const AGXMagneticJointDeviceDesc& desc)
{
    static_cast<AGXMagneticJointDeviceDesc&>(*this) = desc;
}

void AGXMagneticJointDevice::getDesc(AGXMagneticJointDeviceDesc& desc)
{
    desc = static_cast<AGXMagneticJointDeviceDesc&>(*this);
}

const Mapping* AGXMagneticJointDevice::info() const
{
    return m_info;
}

Mapping* AGXMagneticJointDevice::info()
{
    return m_info;
}

void AGXMagneticJointDevice::resetInfo(Mapping* info)
{
    m_info = info;
}

/////////////////////////////////////////////////////////////////////////
// Magnetizer
class JointMagnetizer : public agxSDK::StepEventListener
{
private:
    agx::ConstraintObserver m_joint;
    const double m_validDistance;
    const double m_validAngleRad;
    agx::RigidBodyObserver m_rigids[2];
    agx::FrameObserver m_frames[2];

public:
    JointMagnetizer(agx::Constraint* joint, const double& validDistance = 0.0, const double& validAngleDeg = 0.0) :
        m_joint(joint),
        m_validDistance(validDistance),
        m_validAngleRad(agx::degreesToRadians(validAngleDeg))
    {
        for(size_t i = 0; i < 2; ++i){
            m_rigids[i] = m_joint->getBodyAt(i);
            m_frames[i] = m_joint->getAttachment(i)->getFrame();
        }
    }

   virtual void post( const agx::TimeStamp& t )
    {
        if(m_joint->getEnable()) return;
        agx::AffineMatrix4x4 f_wts[2];  // frame world transforms
        agx::Vec3 f_wdirs[2];           // frame world directions

        for(size_t i = 0; i < 2; ++i){
            f_wts[i] = m_frames[i]->getLocalMatrix() * m_rigids[i]->getTransform();
            f_wdirs[i] = (f_wts[i].getRotate() * agx::Z_AXIS).normal();
        }
        // distance b/w frame0 to frame1
        agx::Vec3 dir = f_wts[1].getTranslate() - f_wts[0].getTranslate();
        const double d = dir.normalize();
        // angle b/w frame0 to frame1
        double rad = acos(f_wdirs[0] * f_wdirs[1]);
        if(abs(d) < m_validDistance && abs(rad) < m_validAngleRad){
            m_joint->setEnable(true);
            m_rigids[0]->getGeometries().front()->setEnableCollisions(m_rigids[1]->getGeometries().front(), false);
        }

        LOGGER_INFO() << "AGXMagneticJoint " << "distance: " << d << " angle: " <<  agx::radiansToDegrees(rad)<< LOGGER_ENDL();
    }
};

/////////////////////////////////////////////////////////////////////////
// AGXMagneticJoint
class AGXMagneticJoint : public AGXBodyExtension
{
public:
    static bool createAGXMagneticJoint(cnoid::AGXBody* agxBody);
    AGXMagneticJoint(AGXMagneticJointDevice* device, AGXBody* agxBody);
private:
    AGXMagneticJointDevicePtr m_device;
};
typedef ref_ptr<AGXMagneticJoint> AGXMagneticJointPtr;


bool AGXMagneticJoint::createAGXMagneticJoint(cnoid::AGXBody* agxBody)
{
    DeviceList<> devices = agxBody->body()->devices();
    DeviceList<AGXMagneticJointDevice> jointDevices;
    jointDevices.extractFrom(devices);
    for(auto device : jointDevices){
        agxBody->addAGXBodyExtension(new cnoid::AGXMagneticJoint(device, agxBody));
    }
    return true;
}

AGXMagneticJoint::AGXMagneticJoint(AGXMagneticJointDevice* device, AGXBody* agxBody) :
    AGXBodyExtension(agxBody),
    m_device(device)
{
    struct AGXMagneticJointParam
    {
        AGXMagneticJointParam(){
            position1 = Vector3();
            position2 = Vector3();
            connectAxis1 = Vector3(0, 0, 1);
            connectAxis2 = Vector3(0, 0, 1);
            validDistance = 0.0;
            validAngle = 0.0;
        }
        string  link1Name, link2Name;
        Vector3 position1;    // joint position on link1 coordinate
        Vector3 position2;    // joint position on link2 coordinate
        Vector3 connectAxis1; // link1 coordinate
        Vector3 connectAxis2; // link2 coordinate
        double validDistance;
        double validAngle;   // degree

    }jp;

    // Get parameters from yaml
    if(!m_device) return;
    Mapping& jointDeviceInfo = *m_device->info();
    jointDeviceInfo.read("link1Name", jp.link1Name);
    jointDeviceInfo.read("link2Name", jp.link2Name);
    agxConvert::setVector(jointDeviceInfo.find("position1"), jp.position1);
    agxConvert::setVector(jointDeviceInfo.find("position2"), jp.position2);
    agxConvert::setVector(jointDeviceInfo.find("connectAxis1"), jp.connectAxis1);
    agxConvert::setVector(jointDeviceInfo.find("connectAxis2"), jp.connectAxis2);
    jointDeviceInfo.read("validDistance", jp.validDistance);
    jointDeviceInfo.read("validAngle", jp.validAngle);
    string constraintType;
    jointDeviceInfo.read("constraintType", constraintType);

    AGXElementaryConstraint base;
    jointDeviceInfo.read("jointCompliance", base.compliance);
    jointDeviceInfo.read("jointSpookDamping", base.spookDamping);

    AGXElementaryConstraint motorDesc;
    jointDeviceInfo.read("jointMotor", motorDesc.enable);
    jointDeviceInfo.read("jointMotorCompliance", motorDesc.compliance);
    jointDeviceInfo.read("jointMotorSpookDamping", motorDesc.spookDamping);
    
    // Create magnetic joint
    AGXLink* const agxLink1 = getAGXBody()->getAGXLink(jp.link1Name);
    AGXLink* const agxLink2 = getAGXBody()->getAGXLink(jp.link2Name);

    if(!agxLink1 || !agxLink2) return;
    Link* const link1 = agxLink1->getOrgLink();
    Link* const link2 = agxLink2->getOrgLink();
    agx::Vec3 axis1z = agxConvert::toAGX(Vector3(0, 0, 1));
    agx::Vec3 axis2z = agxConvert::toAGX(Vector3(0, 0, 1));
    agx::Vec3 axis1 = agxConvert::toAGX(link1->Rs() * jp.connectAxis1);
    agx::Vec3 axis2 = agxConvert::toAGX(link2->Rs() * jp.connectAxis2);
    agx::Vec3 lp1 = agxConvert::toAGX(link1->Rs() * jp.position1);
    agx::Vec3 lp2 = agxConvert::toAGX(link2->Rs() * jp.position2);

    agx::RigidBody* r[2];
    r[0] = getAGXBody()->getAGXRigidBody(jp.link1Name);
    r[1] = getAGXBody()->getAGXRigidBody(jp.link2Name);
    if(!r[0] || !r[1]) return;
    agx::FrameRef f[2];
    f[0] = new agx::Frame();
    f[1] = new agx::Frame();
    f[0]->setLocalTranslate(lp1);
    f[0]->setLocalRotate(agx::Quat(axis1z, axis1));
    f[1]->setLocalTranslate(lp2);
    f[1]->setLocalRotate(agx::Quat(axis2z, axis2));

    agx::Constraint* joint;
    if(constraintType == "hinge"){
        auto hinge = new agx::Hinge(r[0], f[0], r[1], f[1]);
        auto motor = hinge->getMotor1D();
        motor->setEnable(motorDesc.enable);
        motor->setCompliance(motorDesc.compliance);
        motor->setDamping(motorDesc.spookDamping);
        joint = hinge;
    } else {
        joint = new agx::LockJoint(r[0], f[0], r[1], f[1]);
    }

    joint->setEnable(false);
    joint->setCompliance(base.compliance);
    joint->setDamping(base.spookDamping);
    if(!getAGXBody()->getAGXScene()->getSimulation()->add(joint)) return;

    getAGXBody()->getAGXScene()->getSimulation()
        ->add(new JointMagnetizer(joint, jp.validDistance, jp.validAngle));
}

} // cnoid

namespace{
using namespace cnoid;

/////////////////////////////////////////////////////////////////////////
// Register AGXMagneticJointDevice
struct AGXMagneticJointDeviceRegistration
{
    AGXMagneticJointDeviceRegistration()
    {
        YAMLBodyLoader::addNodeType("AGXMagneticJointDevice", AGXMagneticJointDevice::createAGXMagneticJointDevice);
    }
}registrationAGXMagneticJointDevice;

/////////////////////////////////////////////////////////////////////////
// Register AGXMagneticJoint
struct AGXMagneticJointRegistration
{
    AGXMagneticJointRegistration() {
        AGXBody::addAGXBodyExtensionAdditionalFunc("AGXMagneticJoint", AGXMagneticJoint::createAGXMagneticJoint);
    }
}registrationAGXMagneticJoint;

}
