#include "BodyElementOverwriteItem.h"
#include "BodyOverwriteAddon.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/MessageView>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;


void BodyElementOverwriteItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerAbstractClass<BodyElementOverwriteItem>();
}


BodyElementOverwriteItem::BodyElementOverwriteItem()
{
    bodyItem_ = nullptr;
    newBodyItem_ = nullptr;
    isNewBodyItemValid = false;
}


BodyElementOverwriteItem::BodyElementOverwriteItem(const BodyElementOverwriteItem& org)
    : Item(org)
{
    bodyItem_ = nullptr;
    newBodyItem_ = nullptr;
    isNewBodyItemValid = false;
}


BodyOverwriteAddon* BodyElementOverwriteItem::bodyOverwrite()
{
    return bodyItem_ ? bodyItem_->getAddon<BodyOverwriteAddon>() : nullptr;
}


bool BodyElementOverwriteItem::onNewTreePositionCheck(bool isManualOperation, std::function<void()>&)
{
    newBodyItem_ = findOwnerItem<BodyItem>();
    isNewBodyItemValid = true;
    if(newBodyItem_ && (!bodyItem_ || bodyItem_ == newBodyItem_)){
        return onCheckNewOverwritePosition(isManualOperation);
    }
    return false;
}


void BodyElementOverwriteItem::onTreePathChanged()
{
    if(bodyItem_){
        if(!parentItem()){
            onDisconnectedFromBodyItem();
            bodyItem_ = nullptr;
        } else {
            if(!isNewBodyItemValid){
                newBodyItem_ = findOwnerItem<BodyItem>();
            }
            if(!newBodyItem_ || newBodyItem_ != bodyItem_){
                onDisconnectedFromBodyItem();
                bodyItem_ = nullptr;
            }
        }
    }
    newBodyItem_ = nullptr;
    isNewBodyItemValid = false;
}
