/**
   @author Shizuko Hattori
*/

#ifndef CNOID_OPENHRP_PLUGIN_ONLINE_VIEWER_ITEM_H
#define CNOID_OPENHRP_PLUGIN_ONLINE_VIEWER_ITEM_H

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
    virtual Item* doDuplicate() const override;
    virtual void onConnectedToRoot() override;
    virtual void onDisconnectedFromRoot() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
    virtual SgNode* getScene() override;

private:
    OpenHRPOnlineViewerItemImpl* impl;
};

typedef ref_ptr<OpenHRPOnlineViewerItem> OpenHRPOnlineViewerItemPtr;
}

#endif
