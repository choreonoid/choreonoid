#ifndef CNOID_BODY_LINK_KINEMATICS_KIT_H
#define CNOID_BODY_LINK_KINEMATICS_KIT_H

#include <cnoid/EigenTypes>
#include <cnoid/ClonableReferenced>
#include <cnoid/Signal>
#include <cnoid/GeneralId>
#include <memory>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Link;
class JointPath;
class JointSpaceConfigurationHandler;
class CoordinateFrame;
class CoordinateFrameList;
class InverseKinematics;
class Mapping;

class CNOID_EXPORT LinkKinematicsKit : public ClonableReferenced
{
public:
    LinkKinematicsKit(Link* link);
    ~LinkKinematicsKit();

    LinkKinematicsKit* clone(CloneMap* cloneMap = nullptr) const {
        return static_cast<LinkKinematicsKit*>(doClone(cloneMap));
    }

    void setBaseLink(Link* baseLink);
    void setInverseKinematics(std::shared_ptr<InverseKinematics> ik);
    void setBaseFrames(CoordinateFrameList* frames);
    void setOffsetFrames(CoordinateFrameList* frames);

    Body* body();
    const Body* body() const;
    Link* link();
    const Link* link() const;
    Link* baseLink();
    const Link* baseLink() const;
    bool isManipulator() const;
    
    bool hasJointPath() const;
    std::shared_ptr<JointPath> jointPath();
    std::shared_ptr<InverseKinematics> inverseKinematics();
    
    std::shared_ptr<JointSpaceConfigurationHandler> configurationHandler();
    int currentConfigurationType() const;
    std::string configurationLabel(int id) const;

    bool isCustomIkAvaiable() const;
    bool isCustomIkDisabled() const;
    void setCustomIkDisabled(bool on);

    Vector3 referenceRpy() const;
    void setReferenceRpy(const Vector3& rpy);
    void resetReferenceRpy();

    CoordinateFrameList* baseFrames();
    CoordinateFrameList* offsetFrames();
    CoordinateFrame* baseFrame(const GeneralId& id);
    const CoordinateFrame* baseFrame(const GeneralId& id) const;
    CoordinateFrame* offsetFrame(const GeneralId& id);
    const CoordinateFrame* offsetFrame(const GeneralId& id) const;
    const GeneralId& currentBaseFrameId() const;
    const GeneralId& currentOffsetFrameId() const;
    CoordinateFrame* currentBaseFrame();
    const CoordinateFrame* currentBaseFrame() const;
    CoordinateFrame* currentOffsetFrame();
    const CoordinateFrame* currentOffsetFrame() const;
    void setCurrentBaseFrame(const GeneralId& id);
    void setCurrentOffsetFrame(const GeneralId& id);

    //! \note hasJointPath() must be true.
    Isometry3 endPosition(
        const GeneralId& baseFrameId = GeneralId() /* Current */,
        const GeneralId& offsetFrameId = GeneralId() /* Current */) const;

    Isometry3 globalEndPosition(
        const GeneralId& offsetFrameId = GeneralId() /* current */) const;
    
    //! \note hasJointPath() must be true.
    Isometry3 globalBasePosition(const GeneralId& baseFrameId = GeneralId() /* current */) const;

    bool setEndPosition(
        const Isometry3& T,
        const GeneralId& baseFrameId = GeneralId() /* Current */,
        const GeneralId& offsetFrameId = GeneralId() /* Current */,
        int configuration = 0 /* Auto */);

    bool setGlobalEndPosition(
        const Isometry3& T_global,
        const GeneralId& offsetFrameId = GeneralId() /* Current */,
        int configuration = 0 /* Auto */);
    
    /**
       The signal is emitted when there is a change in the current frames or frame list for the target frames.
       Note that updates to the position, note, etc. of each frame are not notified by this signal.
       The updates of each frame can be detected using the CoordinateeFrame::isgUpdated signal.
    */
    SignalProxy<void()> sigFrameSetChange();
    void notifyFrameSetChange();

    SignalProxy<void(const Isometry3& T_frameCoordinate)> sigPositionError();
    void notifyPositionError(const Isometry3& T_frameCoordinate);

    bool storeState(Mapping& archive) const;
    bool restoreState(const Mapping& archive);

protected:
    LinkKinematicsKit(const LinkKinematicsKit& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<LinkKinematicsKit> LinkKinematicsKitPtr;
    
}

#endif
