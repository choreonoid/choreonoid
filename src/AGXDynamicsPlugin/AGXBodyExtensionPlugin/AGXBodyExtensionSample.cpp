/**
   \file
   \author Ikumi Susa
*/

#include "AGXBodyExtensionSample.h"
#include <cnoid/AGXBody>

namespace{
/////////////////////////////////////////////////////////////////////////
// Register AGXBodyExtensionSample
bool createAGXBodyExtensionSample(cnoid::AGXBody* agxBody){
    agxBody->addAGXBodyExtension(new cnoid::AGXBodyExtensionSample(agxBody));
    return true;
}

struct AGXBodyExtensionSampleRegistration{
    AGXBodyExtensionSampleRegistration(){
        cnoid::AGXBody::addAGXBodyExtensionAdditionalFunc("AGXBodyExtensionSample", createAGXBodyExtensionSample);
    }
};
AGXBodyExtensionSampleRegistration registrationAGXBodyExtensionSample;

}

namespace cnoid{
/////////////////////////////////////////////////////////////////////////
// AGXBodyExtensionSample
AGXBodyExtensionSample::AGXBodyExtensionSample(AGXBody* agxBody) :
AGXBodyExtension(agxBody)
{

}

}
