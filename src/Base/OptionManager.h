#ifndef CNOID_BASE_OPTION_MANAGER_H
#define CNOID_BASE_OPTION_MANAGER_H

#include <CLI11.hpp>
#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT OptionManager : public CLI::App
{
public:
    static OptionManager* instance();

    OptionManager(const std::string& appDescription);

    void processOptionsPhase1();
    void processOptionsPhase2();
    
    SignalProxy<void(std::vector<std::string>& inputFiles)> sigInputFileOptionsParsed(int phase = 0);
    SignalProxy<void(OptionManager* om)> sigOptionsParsed(int phase = 0);
};

}

#endif
