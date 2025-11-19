#include "OMPLPlugin.h"
#include <cnoid/MessageOut>
#include <cnoid/Format>
#include <ompl/util/Console.h>
#include "gettext.h"

using namespace cnoid;

namespace {
static OMPLPlugin* pluginInstance = nullptr;
}

namespace cnoid {

// Custom output handler to redirect OMPL messages to Choreonoid's MessageOut
class ChoreonoidOutputHandler : public ompl::msg::OutputHandler
{
    MessageOut* mout;
    bool enabled;

public:
    ChoreonoidOutputHandler(MessageOut* messageOut) : mout(messageOut), enabled(false) {}

    void setMessageOut(MessageOut* messageOut) { mout = messageOut; }
    void setEnabled(bool on) { enabled = on; }

    void log(const std::string& text, ompl::msg::LogLevel level, const char* filename, int line) override
    {
        if(!mout || !enabled) return;
        
        // Remove prefix that OMPL adds (e.g., "Info:    ")
        std::string cleanMsg = text;
        if(cleanMsg.find("Info:    ") == 0) {
            cleanMsg = cleanMsg.substr(9);
        } else if(cleanMsg.find("Warning: ") == 0) {
            cleanMsg = cleanMsg.substr(9);
        } else if(cleanMsg.find("Error:   ") == 0) {
            cleanMsg = cleanMsg.substr(9);
        }
        
        switch(level) {
            case ompl::msg::LOG_ERROR:
                mout->putErrorln(formatR(_("OMPL: {0}"), cleanMsg));
                break;
            case ompl::msg::LOG_WARN:
                mout->putWarningln(formatR(_("OMPL: {0}"), cleanMsg));
                break;
            case ompl::msg::LOG_INFO:
                mout->putln(formatR(_("OMPL: {0}"), cleanMsg));
                break;
            case ompl::msg::LOG_DEBUG:
                // Debug messages are usually very verbose, skip them in release builds
                #ifdef DEBUG
                mout->putln(formatR(_("OMPL [DEBUG]: {0}"), cleanMsg));
                #endif
                break;
            case ompl::msg::LOG_NONE:
            default:
                break;
        }
    }
};

class OMPLPlugin::Impl
{
public:
    std::unique_ptr<ChoreonoidOutputHandler> outputHandler;
    
    Impl() {}
    ~Impl() {}
};

}

OMPLPlugin* OMPLPlugin::instance()
{
    return pluginInstance;
}

OMPLPlugin::OMPLPlugin()
    : Plugin("OMPL"), impl(new Impl())
{
    require("Body");
    pluginInstance = this;
}

OMPLPlugin::~OMPLPlugin()
{
    pluginInstance = nullptr;
}

bool OMPLPlugin::initialize()
{
    // Redirect OMPL log messages to Choreonoid's message system
    // Note: This is a global setting that affects all OMPL usage in the process.
    // If other plugins also use OMPL and set their own handler, it will override this.
    impl->outputHandler = std::make_unique<ChoreonoidOutputHandler>(MessageOut::master());
    ompl::msg::useOutputHandler(impl->outputHandler.get());

    return true;
}

bool OMPLPlugin::finalize()
{
    // Reset to default handler when plugin is unloaded
    if(impl->outputHandler) {
        ompl::msg::useOutputHandler(nullptr);
        impl->outputHandler.reset();
    }
    return true;
}

void OMPLPlugin::setMessageOutputEnabled(bool on)
{
    if(impl->outputHandler) {
        impl->outputHandler->setEnabled(on);
    }
}

void OMPLPlugin::setMessageOut(MessageOut* mout)
{
    if(impl->outputHandler) {
        impl->outputHandler->setMessageOut(mout);
    }
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(OMPLPlugin)
