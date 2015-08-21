/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_DEVICE_H
#define CNOID_BODY_DEVICE_H

#include <cnoid/PolymorphicReferencedArray>
#include <cnoid/EigenTypes>
#include <cnoid/Signal>
#include <string>
#include <typeinfo>
#include "exportdecl.h"

namespace cnoid {

class Link;

class CNOID_EXPORT DeviceState : public Referenced
{
protected:
    DeviceState() { }
    DeviceState(const DeviceState& org) { }
        
public:
    virtual ~DeviceState() { }
    virtual void copyStateFrom(const DeviceState& other) = 0;
    virtual DeviceState* cloneState() const = 0;

    /**
       Size of the double-precision floating numbers for representing the state.
    */
    virtual int stateSize() const = 0;

    /**
       @return The position in the buf after reading.
       The value is used when the super class's readState is called by the inherited class.
    */
    virtual const double* readState(const double* buf) = 0;

    /**
       @return The position in the buf after reading.
       The value is used when the super class's readState is called by the inherited class.
    */
    virtual double* writeState(double* out_buf) const = 0;
};
typedef ref_ptr<DeviceState> DeviceStatePtr;


class CNOID_EXPORT Device : public DeviceState
{
    struct NonState {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        int index; // automatically assigned
        int id; // pre-defined id
        std::string name;
        Link* link;
        Isometry3 T_local;
        double cycle;
        const Isometry3& const_T_local() const { return T_local; }
        Signal<void()> sigStateChanged;
    };
        
    NonState* ns;

protected:
    Device(); 
    Device(const Device& org, bool copyAll = true);

public:
    virtual ~Device();

    void setIndex(int index) { ns->index = index; }
    void setId(int id) { ns->id = id; }
    void setName(const std::string& name) { ns->name = name; }
    void setLink(Link* link) { ns->link = link; }
        
    virtual Device* clone() const = 0;
    virtual void forEachActualType(boost::function<bool(const std::type_info& type)> func);
    virtual void clearState();

    bool hasStateOnly() const { return (ns != 0); }

    const int index() const { return ns->index; }
    const int id() const { return ns->id; }
    const std::string& name() const { return ns->name; }
        
    const Link* link() const { return ns->link; }
    Link* link() { return ns->link; }

    Isometry3& T_local() { return ns->T_local; }
    const Isometry3& T_local() const { return ns->T_local; }
        
    Isometry3::ConstLinearPart R_local() const { return ns->const_T_local().linear(); }
    Isometry3::LinearPart R_local() { return ns->T_local.linear(); }

    Isometry3::ConstLinearPart localRotation() const { return ns->const_T_local().linear(); }
    Isometry3::LinearPart localRotaion() { return ns->T_local.linear(); }

    template<typename Derived>
        void setLocalRotation(const Eigen::MatrixBase<Derived>& R) { ns->T_local.linear() = R; }

    Isometry3::ConstTranslationPart p_local() const { return ns->const_T_local().translation(); }
    Isometry3::TranslationPart p_local() { return ns->T_local.translation(); }

    Isometry3::ConstTranslationPart localTranslation() const { return ns->const_T_local().translation(); }
    Isometry3::TranslationPart localTranslation() { return ns->T_local.translation(); }

    template<typename Derived>
        void setLocalTranslation(const Eigen::MatrixBase<Derived>& p) { ns->T_local.translation() = p; }
        
    double cycle() const { return ns->cycle; }
    void setCycle(double msec) { ns->cycle = msec; }

    SignalProxy<void()> sigStateChanged() {
        return ns->sigStateChanged;
    }

    void notifyStateChange() {
        ns->sigStateChanged();
    }

};

typedef ref_ptr<Device> DevicePtr;


class CNOID_EXPORT ActiveDevice : public Device
{
public:
protected:
    ActiveDevice() { }
    ActiveDevice(const Device& org, bool copyAll = true) : Device(org, copyAll) { }
    virtual void forEachActualType(boost::function<bool(const std::type_info& type)> func);
    void copyStateFrom(const ActiveDevice& other) { Device::copyStateFrom(other); }
};

typedef ref_ptr<ActiveDevice> ActiveDevicePtr;


template <class DeviceType = Device, class PointerType = ref_ptr<DeviceType> >
class DeviceList : public PolymorphicReferencedArray<DeviceType, Device, PointerType>
{
    typedef PolymorphicReferencedArray<DeviceType, Device, PointerType> ArrayBase;

public:
    DeviceList() { }

    template <class RhsDeviceType>
    DeviceList(const DeviceList<RhsDeviceType>& rhs)
        : ArrayBase(rhs) { }

    template <class SubType>
    SubType* get(int index) const { return dynamic_cast<SubType*>(ArrayBase::operator[](index).get()); }

    DeviceType* get(int index) const { return ArrayBase::operator[](index).get(); }

    template<class TargetType> void makeIdMap(DeviceList<TargetType>& out_map) const {
        out_map.clear();
        for(size_t i=0; i < ArrayBase::size(); ++i){
            if(TargetType* device = dynamic_cast<TargetType*>(get(i))){
                const int id = device->id();
                if(id >= 0){
                    if(out_map.size() <= id){
                        out_map.resize(id + 1);
                    }
                    out_map[id] = device;
                }
            }
        }
    }
};

};

#endif
