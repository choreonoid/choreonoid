#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_EXTENSION_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_EXTENSION_H

#include <cnoid/Referenced>
#include "AGXObjectFactory.h"
#include "exportdecl.h"

namespace cnoid{
class AGXBody;
class AGXLink;
class Link;
class CNOID_EXPORT AGXBodyExtension : public Referenced
{
public:
    AGXBodyExtension(AGXBody* agxBody);
    AGXBody* getAGXBody();
    agxSDK::Assembly* getAssembly();
private:
    AGXBody* _agxBody;
    agxSDK::AssemblyRef _assembly;
};
typedef ref_ptr<AGXBodyExtension> AGXBodyExtensionPtr;

class AGXExtraJoint : public AGXBodyExtension {
public:
    AGXExtraJoint(AGXBody* agxBody);
    void createJoints();
};

class AGXContinousTrack : public AGXBodyExtension {
public:
    AGXContinousTrack(AGXLink* footLinkStart, AGXBody* body);
private:
    AGXContinousTrack();
    AGXLink*  _chassisLink;
    std::vector<AGXLink*> _feet;
    void addFoot(Link* link, AGXBody* body);
    void createTrackConstraint();
};

}

#endif
