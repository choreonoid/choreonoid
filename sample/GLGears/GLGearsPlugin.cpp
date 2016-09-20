
#include "GLGearsView.h"
#include <cnoid/Plugin>
#include <cnoid/ViewManager>

using namespace cnoid;

class GLGearsPlugin : public Plugin
{
public:
    
    GLGearsPlugin() : Plugin("GLGears") {
        setUnloadable(true);
    }
    
    virtual bool initialize() {

        viewManager().registerClass<GLGearsView>(
            "GLGearsView", "GL Gears", ViewManager::SINGLE_OPTIONAL);

        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(GLGearsPlugin);
