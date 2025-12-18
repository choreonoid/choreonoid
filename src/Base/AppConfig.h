#ifndef CNOID_BASE_APP_CONFIG_H
#define CNOID_BASE_APP_CONFIG_H

#include <filesystem>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Mapping;

class CNOID_EXPORT AppConfig
{
public:
    static bool initialize(const std::string& application, const std::string& organization);
    static const std::filesystem::path& configDataDirPath();
    static Mapping* archive();
    static bool flush();
};

}

#endif
