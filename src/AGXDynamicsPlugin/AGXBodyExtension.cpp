#include "AGXBodyExtension.h"
#include "AGXBody.h"
#include <iostream>

namespace cnoid{

AGXBodyExtension::AGXBodyExtension(AGXBody* agxBody){
    std::cout << "Hello! This is AGXBodyExtension." << std::endl;
    agxBody->addAGXBodyExtension(this);
}

agxSDK::Assembly* AGXBodyExtension::getAssembly()
{
    return _assembly;
}

}

