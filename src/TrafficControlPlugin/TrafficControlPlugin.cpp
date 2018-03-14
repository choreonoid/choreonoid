/**
   @author Japan Atomic Energy Agency
*/

#include "TrafficControlPluginHeader.h"
#include "DynamicTrafficControlPluginHeader.h"

using namespace std;
using namespace cnoid;
using boost::format;

class TrafficControlPlugin : public Plugin
{
public:
    
    TrafficControlPlugin() : Plugin("TrafficControl"){

    }

    virtual bool initialize() override {
        bool ret = TrafficControlShare::instance()->initialize();

        if(ret==false) {
            return false;
        }

        TrafficControlSimulatorItem::initializeClass(this);
        DynamicTrafficControlSimulatorItem::initializeClass(this);

        return true;
    }

    virtual bool finalize() override {
        return Plugin::finalize();
    }

    virtual const char* description() const override
    {
        static std::string text =
            str(format("TrafficControl Plugin Version %1%\n") % CNOID_FULL_VERSION_STRING) +
            "\n" +
            "Copyrigh (c) 2018 Japan Atomic Energy Agency.\n"
            "\n" +
            MITLicenseText();

        return text.c_str();
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(TrafficControlPlugin)
