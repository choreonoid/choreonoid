#ifndef CNOID_BODY_LINK_KINEMATICS_KIT_H
#define CNOID_BODY_LINK_KINEMATICS_KIT_H

#include <cnoid/LinkCoordFrameSetSuite>
#include <cnoid/CloneableReferenced>
#include <cnoid/Signal>
#include <memory>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Link;
class JointPath;
class JointSpaceConfigurationHandler;
class GeneralId;
class CoordinateFrame;
class CoordinateFrameSet;
class InverseKinematics;

class CNOID_EXPORT LinkKinematicsKit : public CloneableReferenced
{
public:
    LinkKinematicsKit(Link* link);
    ~LinkKinematicsKit();

    void setBaseLink(Link* baseLink);
    void setInversetKinematics(std::shared_ptr<InverseKinematics> ik);
    void setFrameSetSuite(LinkCoordFrameSetSuite* frameSetSuite);

    Body* body();
    Link* link();
    Link* baseLink();
    bool isManipulator() const;
    
    std::shared_ptr<InverseKinematics> inverseKinematics();
    std::shared_ptr<JointPath> jointPath();
    
    std::shared_ptr<JointSpaceConfigurationHandler> configurationHandler();
    int currentConfigurationType() const;
    std::string configurationLabel(int id) const;

    bool isCustomIkAvaiable() const;
    bool isCustomIkDisabled() const;
    void setCustomIkDisabled(bool on);

    Vector3 referenceRpy() const;
    void setReferenceRpy(const Vector3& rpy);
    void resetReferenceRpy();

    enum FrameType {
        WorldFrame = LinkCoordFrameSetSuite::WorldFrame,
        BodyFrame = LinkCoordFrameSetSuite::BodyFrame,
        LinkFrame = LinkCoordFrameSetSuite::LinkFrame
    };

    LinkCoordFrameSetSuite* frameSetSuite();

    CoordinateFrameSet* frameSet(int frameType);
    CoordinateFrameSet* worldFrameSet();
    CoordinateFrameSet* bodyFrameSet();
    CoordinateFrameSet* linkFrameSet();
    
    CoordinateFrame* worldFrame(const GeneralId& id);
    CoordinateFrame* bodyFrame(const GeneralId& id);
    CoordinateFrame* linkFrame(const GeneralId& id);

    const GeneralId& currentFrameId(int frameType) const;
    const GeneralId& currentWorldFrameId() const;
    const GeneralId& currentBodyFrameId() const;
    const GeneralId& currentLinkFrameId() const;
    
    CoordinateFrame* currentFrame(int frameType);
    CoordinateFrame* currentWorldFrame();
    CoordinateFrame* currentBodyFrame();
    CoordinateFrame* currentLinkFrame();

    void setCurrentFrame(int frameType, const GeneralId& id);
    void setCurrentWorldFrame(const GeneralId& id);
    void setCurrentBodyFrame(const GeneralId& id);
    void setCurrentLinkFrame(const GeneralId& id);

    int currentBaseFrameType() const; // WorldFrame or BodyFrame
    void setCurrentBaseFrameType(int frameType);
    const GeneralId& currentBaseFrameId() const;
    CoordinateFrame* currentBaseFrame();
    void setCurrentBaseFrame(const GeneralId& id);
    Position globalBasePosition() const;

    // Any update on frames (frame lists, current frames, etc.)
    SignalProxy<void()> sigFrameUpdate();
    void notifyFrameUpdate();

    SignalProxy<void(const Position& T_frameCoordinate)> sigPositionError();
    void notifyPositionError(const Position& T_frameCoordinate);

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
