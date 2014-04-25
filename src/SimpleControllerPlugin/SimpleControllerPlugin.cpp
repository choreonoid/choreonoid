/*! @file
  @author Shin'ichiro Nakaoka
*/

#include "SimpleControllerItem.h"
#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include "gettext.h"

using namespace cnoid;

namespace {
    
class SimpleControllerPlugin : public Plugin
{
public:
    SimpleControllerPlugin() : Plugin("SimpleController") {
        require("Body");
    }
        
    virtual bool initialize() {

        itemManager().registerClass<SimpleControllerItem>(N_("SimpleControllerItem"));
        itemManager().addCreationPanel<SimpleControllerItem>();

        return true;
    }
        
    virtual bool finalize() {
        return true;
    }
};
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(SimpleControllerPlugin);
