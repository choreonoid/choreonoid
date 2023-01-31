#ifndef CNOID_BODY_PLUGIN_BODY_ELEMENT_OVERWRITE_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_ELEMENT_OVERWRITE_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;
class BodyOverwriteAddon;

/**
   \note All the items that inherit this class works with BodyItem and BodyOverwriteAddon
*/
class CNOID_EXPORT BodyElementOverwriteItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    BodyItem* bodyItem() { return bodyItem_; }
    BodyOverwriteAddon* bodyOverwrite();

    virtual void releaseOverwriteTarget() = 0;
    
protected:
    BodyElementOverwriteItem();
    BodyElementOverwriteItem(const BodyElementOverwriteItem& org, CloneMap* cloneMap);

    void setBodyItem(BodyItem* item) { bodyItem_ = item; }

    //! The next candidate of the target BodyItem when the item position is updated
    BodyItem* newBodyItem() { return newBodyItem_; }

    virtual bool onNewOverwritePositionCheck(bool isManualOperation) = 0;
    virtual void onDisconnectedFromBodyItem() = 0;

private:
    virtual bool onNewTreePositionCheck(
        bool isManualOperation, std::function<void()>& out_callbackWhenAdded) override final;
    virtual void onTreePathChanged() override final;
    
    BodyItem* bodyItem_;
    BodyItem* newBodyItem_;
    bool isNewBodyItemValid;
};

typedef ref_ptr<BodyElementOverwriteItem> BodyElementOverwriteItemPtr;

}

#endif
