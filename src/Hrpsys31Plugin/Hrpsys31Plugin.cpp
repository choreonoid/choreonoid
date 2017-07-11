/**
   @author Shin'ichiro Nakaoka
*/

#include "Hrpsys31Item.h"
#include "RTMPointCloudIOItem.h"
#include <cnoid/Plugin>

using namespace std;
using namespace cnoid;

namespace {
    
class Hrpsys31Plugin : public Plugin
{
public:
    Hrpsys31Plugin();
    virtual bool initialize();
    virtual bool finalize();
};

};


Hrpsys31Plugin::Hrpsys31Plugin()
    : Plugin("Hrpsys31")
{
    require("RobotAccess");
    require("OpenRTM");
}


bool Hrpsys31Plugin::initialize()
{
    Hrpsys31Item::initializeClass(this);
    RTMPointCloudIOItem::initializeClass(this);
    return true;
}


bool Hrpsys31Plugin::finalize()
{
    return true;
}
    

CNOID_IMPLEMENT_PLUGIN_ENTRY(Hrpsys31Plugin)
