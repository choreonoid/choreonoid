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
    //boost::program_options::options_description_easy_init addOptions();

    OptionManager& addOption(const char* name, const char* description);
    OptionManager& addOption(const char* name, const boost::program_options::value_semantic* s);
    OptionManager& addOption(const char* name, const boost::program_options::value_semantic* s, const char* description);
    OptionManager& addPositionalOption(const char* name, int maxCount);

    /**
       @if jp
       コマンドラインオプションがパースされ、オプション解析処理の準備が整ったときに
       発行されるシグナルをハンドラ関数と接続する。
       @endif
    */
    SignalProxy<void(boost::program_options::variables_map& variables)> sigOptionsParsed() {
        return sigOptionsParsed_;
    }

private:
    OptionManager();
    ~OptionManager();

    bool parseCommandLine(int argc, char *argv[]);

    Signal<void(boost::program_options::variables_map& variables)> sigOptionsParsed_;

    friend class ExtensionManager;
    friend class AppImpl;
};

}

#endif
