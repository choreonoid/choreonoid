/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_OPTION_MANAGER_H_INCLUDED
#define CNOID_GUIBASE_OPTION_MANAGER_H_INCLUDED

#include "ExtensionManager.h"
#include <cnoid/SignalProxy>
#include <boost/program_options.hpp>
#include <boost/signal.hpp>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT OptionManager
{
public:
    //boost::program_options::options_description_easy_init addOptions();

    OptionManager& addOption(const char* name, const char* description);
    OptionManager& addOption(const char* name, const boost::program_options::value_semantic* s);
    OptionManager& addOption(const char* name, const boost::program_options::value_semantic* s, const char* description);
    OptionManager& addPositionalOption(const char* name, int maxCount);

    typedef boost::signal<void(boost::program_options::variables_map& variables)> SigOptionsParsed;

    /**
       @if jp
       コマンドラインオプションがパースされ、オプション解析処理の準備が整ったときに
       発行されるシグナルをハンドラ関数と接続する。
       @endif
    */
    inline SignalProxy<SigOptionsParsed> sigOptionsParsed() {
        return sigOptionsParsed_;
    }

private:
    OptionManager();
    ~OptionManager();

    bool parseCommandLine(int argc, char *argv[]);

    SigOptionsParsed sigOptionsParsed_;

    friend class ExtensionManager;
    friend class AppImpl;
};
}

#endif
