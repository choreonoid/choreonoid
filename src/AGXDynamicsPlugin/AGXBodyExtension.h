#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_EXTENSION_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_EXTENSION_H

#include <cnoid/Referenced>
#include "AGXObjectFactory.h"
#include "exportdecl.h"

namespace cnoid{
class Link;
class AGXBody;
class AGXLink;
class CNOID_EXPORT AGXBodyExtension : public Referenced
{
public:
    AGXBodyExtension(AGXBody* agxBody);
    AGXBody* getAGXBody();
private:
    AGXBody* _agxBody;
};
typedef ref_ptr<AGXBodyExtension> AGXBodyExtensionPtr;
typedef std::vector<AGXBodyExtensionPtr> AGXBodyExtensionPtrs;

class AGXExtraJoint : public AGXBodyExtension {
public:
    AGXExtraJoint(AGXBody* agxBody);
    void createJoints();
};

class AGXContinuousTrack : public AGXBodyExtension {
public:
    AGXContinuousTrack(AGXLink* footLinkStart, AGXBody* body);
private:
    AGXContinuousTrack();
    AGXLink*  _chassisLink;
    std::vector<AGXLink*> _feet;
    void addFoot(Link* link, AGXBody* body);
    void createTrackConstraint();
};

}

#endif
