#ifndef CNOID_BODY_BODY_KINEMATICS_KIT_H
#define CNOID_BODY_BODY_KINEMATICS_KIT_H

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
class JointTraverse;
class CoordinateFrame;
class CoordinateFrameList;
class InverseKinematics;
class LinkedJointHandler;
class Mapping;

/**
   This class has the following modes:
   1. End link mode
     Only the end link is specified and other kinematics objects such as the joint path and the
     inverse kinematics object are not specified. This mode is used for situations where the base
     link is not identified.
   2. Joint path mode
     A joint path is specified and the inverse kinematics object is obtained from the path.
   3. Inverse kinematics mode
     An inverse kinematics object is specified and the joint path is not explicitly specified.
     Note that the inverse kinematics may not always have the corresponding joint path,
     such as when there are multiple base links.
   4. Joint traverse mode
     An arbitrary joint traverse including one with multiple end links can be spcecified.
     You can only use the forward kinematics using the joint traverse object in this mode.
*/
class CNOID_EXPORT BodyKinematicsKit : public ClonableReferenced
{
public:
    BodyKinematicsKit();
    ~BodyKinematicsKit();

    BodyKinematicsKit* clone(CloneMap* cloneMap = nullptr) const {
        return static_cast<BodyKinematicsKit*>(doClone(cloneMap));
    }

    /**
       This function sets the end link in the end link mode.
    */
    void setEndLink(Link* endLink);

    /**
       This function sets a joint path and sets the joint path mode.
       The joint traverse from the base link is also set when the joint path is valid.
    */
    void setJointPath(Link* baseLink, Link* endLink);

    /**
       This function sets the inverse kinematics without joint path mode.
       The joint path is also set when the inverse kinematics is for a joint path.
    */
    void setInverseKinematics(Link* endLink, std::shared_ptr<InverseKinematics> ik);

    /**
       This function sets the joint traverse including the all links from the root link.
       The function sets the joint traverse mode.
    */
    void setJointTraverse(Body* body);

    /**
       This function sets the joint traverse including the all links from the specified base link.
       The function sets the joint traverse mode.
    */
    void setJointTraverse(Link* baseLink);

    /**
       This functions sets the joint traverse mode.
    */
    void setJointTraverse(std::shared_ptr<JointTraverse> jointTraverse);

    void setBaseFrames(CoordinateFrameList* frames);
    void setOffsetFrames(CoordinateFrameList* frames);

    Body* body();
    const Body* body() const;

    //! Note that this function returns nullptr when the end link is not specified.
    Link* endLink();
    //! Note that this function returns nullptr when the end link is not specified.
    const Link* endLink() const;

    [[deprecated("Use endLink")]]
    Link* link() { return endLink(); }
    [[deprecated("Use endLink")]]
    const Link* link() const { return endLink(); }
    
    //! Note that this function returns nullptr when the base link is not specified.
    Link* baseLink();
    //! Note that this function returns nullptr when the base link is not specified.
    const Link* baseLink() const;
    
    bool isManipulator() const;
    
    bool hasJointPath() const;

    /**
       The joint path is only available in the joint path mode.
    */
    std::shared_ptr<JointPath> jointPath();

    bool hasJointTraverse() const;

    /**
       The joint traverse is available when the base link is specified,
       and the joint traverse is from the base link. When the joint path
       is specified, the traverse is set toward the end link.
    */
    std::shared_ptr<JointTraverse> jointTraverse();

    int numJoints() const;
    Link* joint(int index);
    const Link* joint(int index) const;
    std::vector<Link*> joints() const;
    
    /**
       The inverse kinematics is available in the joint path mode and the inverse kinematics without joint path mode.
    */
    std::shared_ptr<InverseKinematics> inverseKinematics();

    /**
       The linked joint handler is only available when the body has it.
    */
    LinkedJointHandler* linkedJointHandler();
    
    /**
       The configuration handler is only available when the joint path has it.
    */
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
       The updates of each frame can be detected using the CoordinateeFrame::sigUpdated signal.
    */
    SignalProxy<void()> sigFrameSetChanged();
    void notifyFrameSetChange();

    SignalProxy<void(const Isometry3& T_frameCoordinate)> sigPositionError();
    void notifyPositionError(const Isometry3& T_frameCoordinate);

    bool storeState(Mapping& archive) const;
    bool restoreState(const Mapping& archive);

protected:
    BodyKinematicsKit(const BodyKinematicsKit& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<BodyKinematicsKit> BodyKinematicsKitPtr;

[[deprecated("Use BodyKinematicsKit")]]
typedef BodyKinematicsKit LinkKinematicsKit;

[[deprecated("Use BodyKinematicsKitPtr")]]
typedef ref_ptr<BodyKinematicsKit> LinkKinematicsKitPtr;

}

#endif
