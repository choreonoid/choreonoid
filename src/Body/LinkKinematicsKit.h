#ifndef CNOID_BODY_LINK_KINEMATICS_KIT_H
#define CNOID_BODY_LINK_KINEMATICS_KIT_H

#include <cnoid/Referenced>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Link;
class JointPath;
class JointPathConfigurationHandler;
class GeneralId;
class CoordinateFrame;
class CoordinateFrameSet;
class CoordinateFrameSetPair;
class InverseKinematics;

class CNOID_EXPORT LinkKinematicsKit : public Referenced
{
public:
    LinkKinematicsKit(Link* link);
    ~LinkKinematicsKit();

    void setBaseLink(Link* baseLink);
    void setInversetKinematics(std::shared_ptr<InverseKinematics> ik);
    void setFrameSetPair(CoordinateFrameSetPair* frameSets);

    Body* body();
    Link* link();
    Link* baseLink();
    std::shared_ptr<JointPath> jointPath();
    std::shared_ptr<JointPathConfigurationHandler> configurationHandler();
    int currentConfiguration() const;
    std::string configurationName(int index) const;
    std::shared_ptr<InverseKinematics> inverseKinematics();
    bool isManipulator() const;

    CoordinateFrameSetPair* frameSetPair();
    CoordinateFrameSet* baseFrames();
    CoordinateFrame* baseFrame(const GeneralId& id);
    CoordinateFrameSet* localFrames();
    CoordinateFrame* localFrame(const GeneralId& id);
    const GeneralId& currentBaseFrameId();
    CoordinateFrame* currentBaseFrame();
    void setCurrentBaseFrame(const GeneralId& id);
    const GeneralId& currentLocalFrameId();
    CoordinateFrame* currentLocalFrame();
    void setCurrentLocalFrame(const GeneralId& id);

    Body* findAttachedEndEffector() const;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<LinkKinematicsKit> LinkKinematicsKitPtr;
    
}

#endif
