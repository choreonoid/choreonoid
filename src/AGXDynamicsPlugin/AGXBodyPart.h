#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_PART_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_PART_H

#include <cnoid/Referenced>
#include "AGXObjectFactory.h"

namespace cnoid {

class Link;
typedef ref_ptr<Link> LinkPtr;
class AGXBody;
typedef ref_ptr<AGXBody> AGXBodyPtr;
class AGXLink;
typedef ref_ptr<AGXLink> AGXLinkPtr;
typedef std::vector<AGXLinkPtr> AGXLinkPtrs;

class AGXBodyPart : public Referenced
{
public:
    AGXBodyPart();
    bool hasSelfCollisionGroupName() const;
    std::string getSelfCollisionGroupName() const;
    int numAGXConstraints() const;
    agx::ConstraintRef getAGXConstraint(const int& index) const;
    int numAGXAssemblys() const;
    agxSDK::AssemblyRef getAGXAssembly(const int& index);
protected:
    void setSelfCollsionGroupName(const std::string& name);
    void addAGXConstraint(agx::ConstraintRef const constraint);
    void addAGXAssembly(agxSDK::Assembly* assembly);
    void addAGXLinkedStructure(agxSDK::LinkedStructureRef const structure);
private:
    bool _hasSelfCollisionGroupName;
    std::string _selfCollisionGroupName;
    std::vector<agx::ConstraintRef> _constraints;
    std::vector<agxSDK::AssemblyRef> _assemblys;
    std::vector<agxSDK::LinkedStructureRef> _structures;
};
typedef ref_ptr<AGXBodyPart> AGXBodyPartPtr;

class AGXExtraJoint : public AGXBodyPart {
public:
    AGXExtraJoint(AGXBodyPtr agxBody);
    void createJoints(AGXBodyPtr agxBody);
};

class AGXContinousTrack : public AGXBodyPart {
public:
    AGXContinousTrack(AGXLinkPtr footLinkStart, AGXBodyPtr body);
private:
    AGXLinkPtr  _chassisLink;
    AGXLinkPtrs _feet;
    void addFoot(LinkPtr link, AGXBodyPtr body);
    void createTrackConstraint();
};

}
#endif