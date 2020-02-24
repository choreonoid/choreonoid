#ifndef CNOID_BODY_LINK_KINEMATICS_KIT_H
#define CNOID_BODY_LINK_KINEMATICS_KIT_H

#include <cnoid/LinkCoordinateFrameSet>
#include <cnoid/CloneableReferenced>
#include <cnoid/Signal>
#include <memory>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Link;
class JointPath;
class JointPathConfigurationHandler;
class GeneralId;
class CoordinateFrame;
class CoordinateFrameSet;
class LinkCoordinateFrameSet;
class InverseKinematics;

class CNOID_EXPORT LinkKinematicsKit : public CloneableReferenced
{
public:
    LinkKinematicsKit(Link* link);
    ~LinkKinematicsKit();

    void setBaseLink(Link* baseLink);
    void setInversetKinematics(std::shared_ptr<InverseKinematics> ik);
    void setFrameSets(LinkCoordinateFrameSet* frameSets);

    Body* body();
    Link* link();
    Link* baseLink();
    bool isManipulator() const;
    
    std::shared_ptr<InverseKinematics> inverseKinematics();
    std::shared_ptr<JointPath> jointPath();
    
    std::shared_ptr<JointPathConfigurationHandler> configurationHandler();
    int currentConfiguration() const;
    std::string configurationName(int index) const;

    bool isCustomIkAvaiable() const;
    bool isCustomIkDisabled() const;
    void setCustomIkDisabled(bool on);

    Vector3 referenceRpy() const;
    void setReferenceRpy(const Vector3& rpy);
    void resetReferenceRpy();

    enum FrameType {
        WorldFrame = LinkCoordinateFrameSet::WorldFrame,
        BodyFrame = LinkCoordinateFrameSet::BodyFrame,
        EndFrame = LinkCoordinateFrameSet::EndFrame
    };

    LinkCoordinateFrameSet* frameSets();

    CoordinateFrameSet* frameSet(int frameType);
    CoordinateFrameSet* worldFrameSet();
    CoordinateFrameSet* bodyFrameSet();
    CoordinateFrameSet* endFrameSet();
    
    CoordinateFrame* worldFrame(const GeneralId& id);
    CoordinateFrame* bodyFrame(const GeneralId& id);
    CoordinateFrame* endFrame(const GeneralId& id);

    const GeneralId& currentFrameId(int frameType) const;
    const GeneralId& currentWorldFrameId() const;
    const GeneralId& currentBodyFrameId() const;
    const GeneralId& currentEndFrameId() const;
    
    CoordinateFrame* currentFrame(int frameType);
    CoordinateFrame* currentWorldFrame();
    CoordinateFrame* currentBodyFrame();
    CoordinateFrame* currentEndFrame();

    void setCurrentFrame(int frameType, const GeneralId& id);
    void setCurrentWorldFrame(const GeneralId& id);
    void setCurrentBodyFrame(const GeneralId& id);
    void setCurrentEndFrame(const GeneralId& id);

    int currentBaseFrameType(); // WorldFrame or BodyFrame
    void setCurrentBaseFrameType(int frameType);
    const GeneralId& currentBaseFrameId() const;
    CoordinateFrame* currentBaseFrame();
    void setCurrentBaseFrame(const GeneralId& id);

    // Any update on frames (frame lists, current frames, etc.)
    SignalProxy<void()> sigFrameUpdate();
    void notifyFrameUpdate();

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
