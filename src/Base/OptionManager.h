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

    /**
       Positional option is replaced with input file options that can obtained by the sigInputFileOptionsParsed signal.
    */
    // OptionManager& addPositionalOption(const char* name, int maxCount);

    SignalProxy<void(std::vector<std::string>& inputFiles)> sigInputFileOptionsParsed(int phase = 0);
    SignalProxy<void(boost::program_options::variables_map& variables)> sigOptionsParsed(int phase = 0);

    bool parseCommandLine1(int argc, char *argv[]);
    void parseCommandLine2();

private:
    OptionManager();
    ~OptionManager();

    friend class ExtensionManager;
};

}

#endif
