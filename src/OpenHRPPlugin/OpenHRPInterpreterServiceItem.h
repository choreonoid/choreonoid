/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENHRP_PLUGIN_INTERPRETER_SERVICE_ITEM_H
#define CNOID_OPENHRP_PLUGIN_INTERPRETER_SERVICE_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class OpenHRPInterpreterServiceItemImpl;

class CNOID_EXPORT OpenHRPInterpreterServiceItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    OpenHRPInterpreterServiceItem();
    OpenHRPInterpreterServiceItem(const OpenHRPInterpreterServiceItem& org);
    virtual ~OpenHRPInterpreterServiceItem();

    void setRTCInstanceName(const std::string& name);
    
protected:
    virtual Item* doDuplicate() const override;
    virtual void onConnectedToRoot() override;
    virtual void onPositionChanged() override;
    virtual void onDisconnectedFromRoot() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
        
private:
    OpenHRPInterpreterServiceItemImpl* impl;
};
        
typedef ref_ptr<OpenHRPInterpreterServiceItem> OpenHRPInterpreterServiceItemPtr;
}

#endif
