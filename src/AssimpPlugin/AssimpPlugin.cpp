#include <cnoid/AssimpSceneLoader>
#include <cnoid/Plugin>

class AssimpPlugin : public cnoid::Plugin
{
public:
    AssimpPlugin() : Plugin("Assimp")
    {

    }
        
    virtual bool initialize()
    {
        cnoid::AssimpSceneLoader::initializeClass();
        return true;
    }
        
    virtual bool finalize()
    {
        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(AssimpPlugin);
