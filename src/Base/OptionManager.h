/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_OPTION_MANAGER_H
#define CNOID_BASE_OPTION_MANAGER_H

#include "ExtensionManager.h"
#include <cnoid/Signal>
#include <boost/program_options.hpp>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT OptionManager
{
public:
    OptionManager& addOption(const char* name, const char* description);
    OptionManager& addOption(const char* name, const boost::program_options::value_semantic* s);
    OptionManager& addOption(const char* name, const boost::program_options::value_semantic* s, const char* description);
    OptionManager& addPositionalOption(const char* name, int maxCount);

    SignalProxy<void(boost::program_options::variables_map& variables)> sigOptionsParsed(int phase = 0);

private:
    OptionManager();
    ~OptionManager();

    bool parseCommandLine1(int argc, char *argv[]);
    void parseCommandLine2();

    friend class ExtensionManager;
    friend class AppImpl;
};

}

#endif
