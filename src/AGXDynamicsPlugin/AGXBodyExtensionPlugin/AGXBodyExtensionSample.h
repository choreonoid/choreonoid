/**
   \file
   \author Ikumi Susa
*/

#ifndef CNOID_AGXBODYEXTENSION_PLUGIN_AGX_BODY_EXTENSION_SAMPLE_H
#define CNOID_AGXBODYEXTENSION_PLUGIN_AGX_BODY_EXTENSION_SAMPLE_H

#include <cnoid/AGXBodyExtension>

namespace cnoid{

class AGXBody;
class AGXBodyExtensionSample : public AGXBodyExtension
{
public:
    AGXBodyExtensionSample(AGXBody* agxBody);
};
typedef ref_ptr<AGXBodyExtensionSample> AGXBodyExtensionSamplePtr;

}

#endif

