
#include "GLGearsView.h"
#include <cnoid/Plugin>

using namespace cnoid;

class GLGearsPlugin : public Plugin
{
public:
    
    GLGearsPlugin() : Plugin("GLGears") { }
    
    virtual bool initialize() {

        addView(new GLGearsView());
            
        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(GLGearsPlugin);
