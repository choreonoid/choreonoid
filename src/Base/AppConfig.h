/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_APP_CONFIG_H
#define CNOID_BASE_APP_CONFIG_H

#include <string>
#include <cnoid/ValueTree>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AppConfig
{
public:
    static bool initialize(const std::string& application, const std::string& organization);
    static Mapping* archive();
    static bool flush();
};

}

#endif
