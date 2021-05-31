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
    setAttribute(Item::Attached);
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
    return bodyItem_->getAddon<BodyOverwriteAddon>();
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


bool BodyElementOverwriteItem::onCheckNewOverwritePosition(bool isManualOperation)
{
    return true;
}


void BodyElementOverwriteItem::onTreePathChanged()
{
    if(bodyItem_){
        if(!parentItem()){
            onDisconnectedFromBodyItem();
        } else {
            if(!isNewBodyItemValid){
                newBodyItem_ = findOwnerItem<BodyItem>();
            }
            if(!newBodyItem_ || newBodyItem_ != bodyItem_){
                onDisconnectedFromBodyItem();
            }
        }
    }
    newBodyItem_ = nullptr;
    isNewBodyItemValid = false;
}


void BodyElementOverwriteItem::onDisconnectedFromBodyItem()
{

}
