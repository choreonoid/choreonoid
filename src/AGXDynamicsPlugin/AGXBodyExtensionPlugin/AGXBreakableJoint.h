/**
   \file
   \author Ikumi Susa
*/

#ifndef AGX_BREAKABLE_JOINT_H
#define AGX_BREAKABLE_JOINT_H

#include <cnoid/Device>
#include <cnoid/AGXBodyExtension>

namespace cnoid{

struct AGXBreakableJointDeviceDesc
{
    AGXBreakableJointDeviceDesc(){
        breakLimitForce = std::numeric_limits<double>::max();
        jointType = "revolute";
    }
    std::string     link1Name;
    std::string     link2Name;
    Vector3         link1LocalPos;
    Vector3         link2LocalPos;
    std::string     jointType;
    Vector3         jointAxis;
    double          breakLimitForce;
};

class AGXBreakableJointDevice : private AGXBreakableJointDeviceDesc, public Device
{
public:
    // Constructor, override
    AGXBreakableJointDevice(const AGXBreakableJointDeviceDesc& desc);
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
    //void initialize();
};
typedef ref_ptr<AGXBreakableJointDevice> AGXBreakableJointDevicePtr;

class AGXBody;
class AGXBreakableJoint : public AGXBodyExtension
{
private:
    AGXBreakableJointDevicePtr m_device;
public:
    AGXBreakableJoint(AGXBreakableJointDevice* device, AGXBody* agxBody);
};
typedef ref_ptr<AGXBreakableJoint> AGXBreakableJointPtr;

}



#endif
