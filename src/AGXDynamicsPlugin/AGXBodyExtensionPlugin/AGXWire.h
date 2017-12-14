/**
   \file
   \author Ikumi Susa
*/

#ifndef AGX_WIRE_H
#define AGX_WIRE_H

#include <cnoid/Device>
#include <cnoid/AGXBodyExtension>
#include <cnoid/ValueTree>

namespace cnoid{

struct AGXWireDeviceDesc
{
    AGXWireDeviceDesc(){
        radius = 0.2;
    }
    double radius;
};

struct WireNodeState{
    Vector3 position;
};
typedef std::vector<WireNodeState> WireNodeStates;

class YAMLBodyLoader;
class AGXWireDevice : private AGXWireDeviceDesc, public Device
{
public:
    static bool createAGXWireDevice(YAMLBodyLoader& loader, Mapping& node);
    AGXWireDevice(const AGXWireDeviceDesc& desc, Mapping* info);
    AGXWireDevice(const AGXWireDevice& org, bool copyStateOnly = false);
    virtual const char* typeName() override;
    void copyStateFrom(const AGXWireDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual Device* clone() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    void setDesc(const AGXWireDeviceDesc& desc);
    void getDesc(AGXWireDeviceDesc& desc);
    const Mapping* info() const;
    Mapping* info();
    void resetInfo(Mapping* info);
    void   setWireRadius(const double& r);
    double getWireRadius();
    void addWireNodeState(const Vector3& pos);
    WireNodeStates& getWireNodeStates();
private:
    MappingPtr m_info;
    WireNodeStates m_wireNodeStates;
    AGXWireDevice();
};
typedef ref_ptr<AGXWireDevice> AGXWireDevicePtr;

class AGXBody;
class AGXWire : public AGXBodyExtension
{
public:
    static bool createAGXWire(AGXBody* agxBody);
    AGXWire(AGXWireDevice* device, AGXBody* agxBody);
    void updateWireNodeStates();
private:
    AGXWireDevicePtr m_device;
    agxWire::WireRef m_wire;
    agxWire::WireWinchControllerRef m_winch;
};
typedef ref_ptr<AGXWire> AGXWirePtr;

}


#endif
