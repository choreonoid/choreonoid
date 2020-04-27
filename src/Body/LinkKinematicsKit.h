#ifndef CNOID_BODY_LINK_KINEMATICS_KIT_H
#define CNOID_BODY_LINK_KINEMATICS_KIT_H

#include <cnoid/EigenTypes>
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
class CoordinateFrameList;
class InverseKinematics;
class Mapping;

class CNOID_EXPORT LinkKinematicsKit : public CloneableReferenced
{
public:
    LinkKinematicsKit(Link* link);
    ~LinkKinematicsKit();

    void setBaseLink(Link* baseLink);
    void setInversetKinematics(std::shared_ptr<InverseKinematics> ik);
    void setBaseFrames(CoordinateFrameList* frames);
    void setOffsetFrames(CoordinateFrameList* frames);

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

    CoordinateFrameList* baseFrames();
    CoordinateFrameList* offsetFrames();
    CoordinateFrame* baseFrame(const GeneralId& id);
    CoordinateFrame* offsetFrame(const GeneralId& id);
    const GeneralId& currentBaseFrameId() const;
    const GeneralId& currentOffsetFrameId() const;
    CoordinateFrame* currentBaseFrame();
    CoordinateFrame* currentOffsetFrame();
    void setCurrentBaseFrame(const GeneralId& id);
    void setCurrentOffsetFrame(const GeneralId& id);

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
