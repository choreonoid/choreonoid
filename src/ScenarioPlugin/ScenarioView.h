/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SCENARIO_PLUGIN_SCENARIO_VIEW_H_INCLUDED
#define CNOID_SCENARIO_PLUGIN_SCENARIO_VIEW_H_INCLUDED

#include <cnoid/View>

namespace cnoid {

class ScenarioViewImpl;
    
class ScenarioView : public cnoid::View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    ScenarioView();
    virtual ~ScenarioView();
        
private:
    friend class ScenarioViewImpl;
    ScenarioViewImpl* impl;
};
}

#endif
