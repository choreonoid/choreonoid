#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGXBODY_EXTENSION_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGXBODY_EXTENSION_H

#include <cnoid/Referenced>
#include "AGXObjectFactory.h"
#include "exportdecl.h"

namespace cnoid{
class AGXBody;
class CNOID_EXPORT AGXBodyExtension : public Referenced
{
public:
    AGXBodyExtension(AGXBody* agxBody);
    agxSDK::Assembly* getAssembly();
private:
    agxSDK::AssemblyRef _assembly;

};
typedef ref_ptr<AGXBodyExtension> AGXBodyExtensionPtr;

}

#endif
