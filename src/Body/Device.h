/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_DEVICE_H
#define CNOID_BODY_DEVICE_H

#include <cnoid/ClonableReferenced>
#include <cnoid/EigenTypes>
#include <cnoid/Signal>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Link;

class CNOID_EXPORT DeviceState : public ClonableReferenced
{
protected:
    DeviceState() { }
    DeviceState(const DeviceState&) { }
        
public:
    virtual ~DeviceState() { }

    virtual const char* typeName() const = 0;
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
        const Isometry3& const_T_local() const { return T_local; }
        Signal<void()> sigStateChanged;
        Signal<void(double time)> sigTimeChanged;
    };
        
    NonState* ns;

protected:
    Device(); 
    Device(const Device& org, bool copyStateOnly = false);
    void copySpecFrom(const Device* other);

public:
    virtual ~Device();

    Device* clone() const {
        return static_cast<Device*>(doClone(nullptr));
    }
    Device* clone(CloneMap& cloneMap) const {
        return static_cast<Device*>(doClone(&cloneMap));
    }

    virtual bool copyFrom(const Device* other);

    void setIndex(int index) { ns->index = index; }
    void setId(int id) { ns->id = id; }
    void setName(const std::string& name) { ns->name = name; }
    void setLink(Link* link) { ns->link = link; }

    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func);

    bool hasStateOnly() const { return (ns != 0); }

    int index() const { return ns->index; }
    int id() const { return ns->id; }
    const std::string& name() const { return ns->name; }

    const Link* link() const { return ns->link; }
    Link* link() { return ns->link; }

    const Body* body() const;
    Body* body();
    
    Isometry3& T_local() { return ns->T_local; }
    const Isometry3& T_local() const { return ns->T_local; }
    const Isometry3& localPosition() const { return ns->T_local; }
    template<class Scalar, int Mode, int Options>
    void setLocalPosition(const Eigen::Transform<Scalar, 3, Mode, Options>& T) {
        ns->T_local = T.template cast<Isometry3::Scalar>();
    }

    Isometry3::ConstLinearPart R_local() const { return ns->const_T_local().linear(); }
    Isometry3::LinearPart R_local() { return ns->T_local.linear(); }
    Isometry3::ConstLinearPart localRotation() const { return ns->const_T_local().linear(); }
    template<typename Derived>
    void setLocalRotation(const Eigen::MatrixBase<Derived>& R) { ns->T_local.linear() = R; }

    Isometry3::ConstTranslationPart p_local() const { return ns->const_T_local().translation(); }
    Isometry3::TranslationPart p_local() { return ns->T_local.translation(); }
    Isometry3::ConstTranslationPart localTranslation() const { return ns->const_T_local().translation(); }
    template<typename Derived>
    void setLocalTranslation(const Eigen::MatrixBase<Derived>& p) { ns->T_local.translation() = p; }

    virtual void clearState();

    virtual bool on() const;
    virtual void on(bool on);

    SignalProxy<void()> sigStateChanged() {
        return ns->sigStateChanged;
    }

    void notifyStateChange() {
        ns->sigStateChanged();
    }

    SignalProxy<void(double time)> sigTimeChanged() {
        return ns->sigTimeChanged;
    }

    void notifyTimeChange(double time) {
        ns->sigTimeChanged(time);
    }

    [[deprecated("Please use T_local() instead")]]
    const Isometry3& T_local_org() const { return ns->T_local; };
    [[deprecated]]
    void setLocalAttitude(const Isometry3& Ta);
    [[deprecated]]
    double cycle() const { return 20.0; }
    [[deprecated]]
    void setCycle(double msec) { }
};

typedef ref_ptr<Device> DevicePtr;

}

#endif
