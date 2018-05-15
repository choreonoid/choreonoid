/*!
  @file
  @author Shizuko Hattori, Shin'ichiro Nakaoka
*/

#include "AssimpSceneLoader.h"
#include <cnoid/Plugin>

using namespace std;
using namespace cnoid;

namespace {
    
class AssimpPlugin : public Plugin
{
public:
    AssimpPlugin() : Plugin("Assimp")
    {

    }
        
    virtual bool initialize()
    {
        AssimpSceneLoader::initializeClass();
        return true;
    }
        
    virtual bool finalize()
    {
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(AssimpPlugin);
