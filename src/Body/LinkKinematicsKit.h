#ifndef CNOID_BODY_LINK_KINEMATICS_KIT_H
#define CNOID_BODY_LINK_KINEMATICS_KIT_H

#include <cnoid/Referenced>
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
    CoordinateFrameSet* frameSet(int which);
    CoordinateFrameSet* baseFrameSet();
    CoordinateFrameSet* localFrameSet();
    
    CoordinateFrame* baseFrame(const GeneralId& id);
    CoordinateFrame* localFrame(const GeneralId& id);

    const GeneralId& currentFrameId(int which);
    const GeneralId& currentBaseFrameId();
    const GeneralId& currentLocalFrameId();
    
    CoordinateFrame* currentBaseFrame();
    CoordinateFrame* currentLocalFrame();

    void setCurrentFrame(int which, const GeneralId& id);
    void setCurrentBaseFrame(const GeneralId& id);
    void setCurrentLocalFrame(const GeneralId& id);

    Body* findAttachedEndEffector() const;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<LinkKinematicsKit> LinkKinematicsKitPtr;
    
}

#endif
