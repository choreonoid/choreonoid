/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>

using namespace cnoid;

class PCLPlugin : public Plugin
{
public:
    
    PCLPlugin() : Plugin("PCL") { }
    
    virtual bool initialize(){

        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(PCLPlugin);
