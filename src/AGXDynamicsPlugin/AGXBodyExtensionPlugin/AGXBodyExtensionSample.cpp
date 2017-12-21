/**
   \file
   \author Ikumi Susa
*/

#include <cnoid/AGXBodyExtension>
#include <cnoid/AGXBody>

namespace cnoid{

/////////////////////////////////////////////////////////////////////////
// AGXBodyExtensionSample
class AGXBodyExtensionSample : public AGXBodyExtension
{
public:
    static bool createAGXBodyExtensionSample(cnoid::AGXBody* agxBody);
    AGXBodyExtensionSample(AGXBody* agxBody);
};
typedef ref_ptr<AGXBodyExtensionSample> AGXBodyExtensionSamplePtr;

bool AGXBodyExtensionSample::createAGXBodyExtensionSample(AGXBody* agxBody){
    agxBody->addAGXBodyExtension(new AGXBodyExtensionSample(agxBody));
    return true;
}

AGXBodyExtensionSample::AGXBodyExtensionSample(AGXBody* agxBody) :
    AGXBodyExtension(agxBody)
{
}

}

namespace{
using namespace cnoid;

/////////////////////////////////////////////////////////////////////////
// Register AGXBodyExtensionSample
struct AGXBodyExtensionSampleRegistration{
    AGXBodyExtensionSampleRegistration(){
        AGXBody::addAGXBodyExtensionAdditionalFunc("AGXBodyExtensionSample", AGXBodyExtensionSample::createAGXBodyExtensionSample);
    }
}registrationAGXBodyExtensionSample;

}
