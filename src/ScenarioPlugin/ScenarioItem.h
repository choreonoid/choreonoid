/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SCENARIO_PLUGIN_SCENARIO_ITEM_H_INCLUDED
#define CNOID_SCENARIO_PLUGIN_SCENARIO_ITEM_H_INCLUDED

#include <cnoid/Item>
#include <cnoid/BodyMotionItem>

namespace cnoid {

class ScenarioItemImpl;

class ScenarioItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    ScenarioItem();
    ScenarioItem(const ScenarioItem& org);
    virtual ~ScenarioItem();
        
protected:
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
};
    
typedef ref_ptr<ScenarioItem> ScenarioItemPtr;
}

#endif
