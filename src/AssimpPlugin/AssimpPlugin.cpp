/*!
  @file
  @author Shizuko Hattori, Shin'ichiro Nakaoka
*/

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
        return true;
    }
        
    virtual bool finalize()
    {
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(AssimpPlugin);
