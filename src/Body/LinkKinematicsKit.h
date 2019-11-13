#ifndef CNOID_BODY_LINK_KINEMATICS_KIT_H
#define CNOID_BODY_LINK_KINEMATICS_KIT_H

#include <cnoid/LinkCoordinateFrameSet>
#include <cnoid/Referenced>
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

class CNOID_EXPORT LinkKinematicsKit : public Referenced
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
    std::shared_ptr<InverseKinematics> inverseKinematics();
    std::shared_ptr<JointPath> jointPath();
    std::shared_ptr<JointPathConfigurationHandler> configurationHandler();
    int currentConfiguration() const;
    std::string configurationName(int index) const;
    bool isManipulator() const;

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

    const GeneralId& currentFrameId(int frameType);
    const GeneralId& currentWorldFrameId();
    const GeneralId& currentBodyFrameId();
    const GeneralId& currentEndFrameId();
    
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
    const GeneralId& currentBaseFrameId();
    CoordinateFrame* currentBaseFrame();
    void setCurrentBaseFrame(const GeneralId& id);

    SignalProxy<void()> sigCurrentFrameChanged();
    void notifyCurrentFrameChange();

    Body* findAttachedEndEffector() const;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<LinkKinematicsKit> LinkKinematicsKitPtr;
    
}

#endif
