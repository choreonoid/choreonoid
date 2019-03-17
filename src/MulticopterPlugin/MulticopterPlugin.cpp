/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"
#include <fmt/format.h>

using namespace std;
using namespace cnoid;
using namespace Multicopter;

class MulticopterPlugin : public Plugin
{
public:
    
    MulticopterPlugin() : Plugin("Multicopter"){
        require("Body");
    }

    virtual bool initialize() override {
        
        bool ret;

        MulticopterSimulatorItem::initializeClass(this);
        
        ret = EventManager::instance()->initialize();
        if( ret == false){
            return ret;
        }

        ret = LinkManager::instance()->initialize();
        if( ret == false){
            return ret;
        }

        ret = SimulationManager::instance()->initialize(this);
        if( ret == false){
            return ret;
        }

        return true;
    }

    virtual bool finalize() override {
        return Plugin::finalize();
    }

    virtual const char* description() const override
    {
        static std::string text =
            fmt::format("Multicoper Plugin Version {}\n", CNOID_FULL_VERSION_STRING) +
            "\n" +
            "Copyrigh (c) 2018 Japan Atomic Energy Agency.\n"
            "\n" +
            MITLicenseText();
        return text.c_str();
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(MulticopterPlugin);
