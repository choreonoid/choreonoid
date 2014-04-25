#include "SceneGraphView.h"
#include "SceneGraphPropertyView.h"
#include <cnoid/Plugin>

using namespace cnoid;

class SceneEditPlugin : public Plugin
{
public:
    
    SceneEditPlugin() : Plugin("SceneEdit") { }
    
    virtual bool initialize() {

        SceneGraphView::initializeClass(this);
        SceneGraphPropertyView::initializeClass(this);
            
        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(SceneEditPlugin);
