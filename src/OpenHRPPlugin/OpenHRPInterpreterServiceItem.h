/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENHRP_INTERPRETER_SERVICE_PLUGIN_INTERPRETER_SERVICE_ITEM_H_INCLUDED
#define CNOID_OPENHRP_INTERPRETER_SERVICE_PLUGIN_INTERPRETER_SERVICE_ITEM_H_INCLUDED

#include <cnoid/Item>

namespace cnoid {

class OpenHRPInterpreterServiceItemImpl;

class OpenHRPInterpreterServiceItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    OpenHRPInterpreterServiceItem();
    OpenHRPInterpreterServiceItem(const OpenHRPInterpreterServiceItem& org);
    virtual ~OpenHRPInterpreterServiceItem();
    
protected:
    virtual ItemPtr doDuplicate() const;
    virtual void onConnectedToRoot();
    virtual void onDisconnectedFromRoot();
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
        
private:
    OpenHRPInterpreterServiceItemImpl* impl;
};
        
typedef ref_ptr<OpenHRPInterpreterServiceItem> OpenHRPInterpreterServiceItemPtr;
}

#endif
