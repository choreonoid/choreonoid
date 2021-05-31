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

protected:
    BodyElementOverwriteItem();
    BodyElementOverwriteItem(const BodyElementOverwriteItem& org);

    void setBodyItem(BodyItem* item) { bodyItem_ = item; }

    //! The next candidate of the target BodyItem when the item position is updated
    BodyItem* newBodyItem() { return newBodyItem_; }

    virtual bool onNewTreePositionCheck(
        bool isManualOperation, std::function<void()>& out_callbackWhenAdded) override final;
    virtual void onTreePathChanged() override final;
    virtual bool onCheckNewOverwritePosition(bool isManualOperation);
    virtual void onDisconnectedFromBodyItem();

private:
    BodyItem* bodyItem_;
    BodyItem* newBodyItem_;
    bool isNewBodyItemValid;
};

typedef ref_ptr<BodyElementOverwriteItem> BodyElementOverwriteItemPtr;

}

#endif
