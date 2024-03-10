#include "OptionManager.h"
#include "MessageView.h"
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

OptionManager* instance_ = nullptr;
vector<string> inputFiles;
Signal<void(std::vector<std::string>& inputFiles)> sigInputFileOptionsParsed_[2];
Signal<void(OptionManager* om)> sigOptionsParsed_[2];

}


OptionManager* OptionManager::instance()
{
    return instance_;
}


OptionManager::OptionManager(const std::string& appDescription)
    : CLI::App(appDescription)
{
    add_option("input-file,--input-file", inputFiles, "general input file");
    instance_ = this;
}


void OptionManager::processOptionsPhase1()
{
    sigOptionsParsed_[0](this);

    if(!inputFiles.empty()){
        sigInputFileOptionsParsed_[0](inputFiles);
    }
}


void OptionManager::processOptionsPhase2()
{
    if(!inputFiles.empty()){
        sigInputFileOptionsParsed_[1](inputFiles);
    }
    
    sigOptionsParsed_[1](this);
    
    // The destructors of the OptionInfo members should be executed here
    // because their elements may be driven by the code instantiated in the plug-in dlls.
    // If the destructors are called when the program is finished after the plug-ins are
    // unloaded, the destructors may cause the segmentation fault
    // by calling the non-existent codes.
    //delete info;
    //info = nullptr;

    for(int i=0; i < 2; ++i){
        sigOptionsParsed_[i].disconnectAllSlots();
    }

    for(auto& file : inputFiles){
        MessageView::instance()->putln(
            fmt::format(_("Input file \"{}\" was not processed."), file),
            MessageView::Warning);
    }
}
    


SignalProxy<void(std::vector<std::string>& inputFiles)> OptionManager::sigInputFileOptionsParsed(int phase)
{
    if(phase < 0){
        phase = 0;
    } else if(phase > 1){
        phase = 1;
    }
    return sigInputFileOptionsParsed_[phase];
}


SignalProxy<void(OptionManager* om)> OptionManager::sigOptionsParsed(int phase)
{
    if(phase < 0){
        phase = 0;
    } else if(phase > 1){
        phase = 1;
    }
    return sigOptionsParsed_[phase];
}
