/**
   @author Shizuko Hattori
*/

#ifndef CNOID_OPENHRP_PLUGIN_ONLINE_VIEWER_ITEM_H_INCLUDED
#define CNOID_OPENHRP_PLUGIN_ONLINE_VIEWER_ITEM_H_INCLUDED

#include <cnoid/Item>
#include <cnoid/SceneProvider>

namespace cnoid {

class OpenHRPOnlineViewerItemImpl;

class OpenHRPOnlineViewerItem : public Item, public SceneProvider
{
public:
    static void initializeClass(ExtensionManager* ext);

    OpenHRPOnlineViewerItem();
    OpenHRPOnlineViewerItem(const OpenHRPOnlineViewerItem& org);
    virtual ~OpenHRPOnlineViewerItem();

protected:
    virtual Item* doDuplicate() const;
    virtual void onConnectedToRoot();
    //virtual void onPositionChanged(){};
    virtual void onDisconnectedFromRoot();
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

    virtual SgNode* getScene();

private:
    OpenHRPOnlineViewerItemImpl* impl;
};

typedef ref_ptr<OpenHRPOnlineViewerItem> OpenHRPOnlineViewerItemPtr;
}

#endif
