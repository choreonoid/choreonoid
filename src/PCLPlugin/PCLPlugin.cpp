/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "MultiPointSetItem.h"
#include <cnoid/Plugin>

using namespace cnoid;

class PCLPlugin : public Plugin
{
public:
    
    PCLPlugin() : Plugin("PCL") { }
    
    virtual bool initialize(){

        MultiPointSetItem::initializeClass(this);
        
        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(PCLPlugin);
