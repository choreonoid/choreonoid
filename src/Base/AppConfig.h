/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_APP_CONFIG_H_INCLUDED
#define CNOID_GUIBASE_APP_CONFIG_H_INCLUDED

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
    static bool save(const std::string& filename);
    static bool load(const std::string& filename);
};
}

#endif
