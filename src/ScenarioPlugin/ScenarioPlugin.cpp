
/*! @file
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include "ScenarioItem.h"
#include "ScenarioView.h"

using namespace cnoid;

namespace {

class ScenarioPlugin : public Plugin
{
public:
        
    ScenarioPlugin() : Plugin("Scenario") {
        require("Body");
        require("Media");
    }

    virtual bool initialize() {
            
        ScenarioItem::initializeClass(this);
        ScenarioView::initializeClass(this);
            
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(ScenarioPlugin);
