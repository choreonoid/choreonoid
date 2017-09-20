#include "AGXBodyExtensionSample.h"
#include "../AGXBody.h"

namespace {

bool createSample(cnoid::AGXBody* agxBody)
{
    cnoid::AGXBodyExtensionPtr ptr = new cnoid::AGXBodyExtension(agxBody); 
    if(ptr){
        agxBody->addAGXBodyExtension(ptr);
        return true;
    }
    return false;
};

struct TypeRegistration
{
    TypeRegistration() {
        cnoid::AGXBody::addAGXBodyExtensionAdditionalFunc("AGXBodyExtensionSample", createSample);
    }
};
TypeRegistration registration;

}