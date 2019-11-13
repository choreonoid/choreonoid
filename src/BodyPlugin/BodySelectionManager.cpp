#include "BodySelectionManager.h"
#include "BodyItem.h"
#include <cnoid/Link>
#include <cnoid/ExtensionManager>
#include <cnoid/TargetItemPicker>
#include <cnoid/ConnectionSet>
#include <cnoid/Archive>
#include <unordered_map>

using namespace std;
using namespace cnoid;

namespace {

struct BodyItemInfo : public Referenced
{
    vector<bool> linkSelection;
    Signal<void(const std::vector<bool>& selection)> sigLinkSelectionChanged;
    ScopedConnection itemDisconnectConnection;
};
typedef ref_ptr<BodyItemInfo> BodyItemInfoPtr;

}

namespace cnoid {

class BodySelectionManager::Impl
{
public:
    TargetItemPicker<BodyItem> targetBodyItemPicker;
    ItemList<BodyItem> selectedBodyItems;
    BodyItemPtr currentBodyItem;
    BodyItemInfoPtr currentInfo;
    unordered_map<BodyItemPtr, BodyItemInfoPtr> bodyItemInfoMap;

    Signal<void(BodyItem* bodyItem, Link* link)> sigCurrentSpecified;
    Signal<void(BodyItem* bodyItem)> sigCurrentBodySpecified;
    Signal<void(BodyItem* bodyItem, Link* link)> sigCurrentChanged;
    Signal<void(BodyItem* bodyItem)> sigCurrentBodyChanged;
    Signal<void(const ItemList<BodyItem>& bodyItems)> sigSelectedBodyItemsChanged;

    Impl();
    Link* getCurrentLink();
    void setCurrentBodyItem(BodyItem* bodyItem, Link* link);
    BodyItemInfo* getOrCreateBodyItemInfo(BodyItem* bodyItem, Link* link);
    void onBodyItemDisconnectedFromRoot(BodyItem* bodyItem);
    bool storeState(Archive& archive);
    void restoreState(const Archive& archive);
};

}

void BodySelectionManager::initializeClass(ExtensionManager* ext)
{
    ext->setProjectArchiver(
        "BodySelectionManager",
        [](Archive& archive){
            return BodySelectionManager::instance()->impl->storeState(archive); },
        [](const Archive& archive){
            return BodySelectionManager::instance()->impl->restoreState(archive); });
}


BodySelectionManager* BodySelectionManager::instance()
{
    static BodySelectionManager* instance_ = new BodySelectionManager;
    return instance_;
}

    
BodySelectionManager::BodySelectionManager()
{
    impl = new Impl;
}


BodySelectionManager::Impl::Impl()
{
    targetBodyItemPicker.sigTargetItemSpecified().connect(
        [&](BodyItem* bodyItem){ setCurrentBodyItem(bodyItem, nullptr); });
}


BodySelectionManager::~BodySelectionManager()
{
    delete impl;
}


SignalProxy<void(BodyItem* bodyItem, Link* link)> BodySelectionManager::sigCurrentSpecified()
{
    return impl->sigCurrentSpecified;
}


SignalProxy<void(BodyItem* bodyItem)> BodySelectionManager::sigCurrentBodySpecified()
{
    return impl->sigCurrentBodySpecified;
}


SignalProxy<void(BodyItem* bodyItem, Link* link)> BodySelectionManager::sigCurrentChanged()
{
    return impl->sigCurrentChanged;
}


SignalProxy<void(BodyItem* bodyItem)> BodySelectionManager::sigCurrentBodyChanged()
{
    return impl->sigCurrentBodyChanged;
}


BodyItem* BodySelectionManager::currentBodyItem()
{
    return impl->currentBodyItem;
}


Link* BodySelectionManager::currentLink()
{
    return impl->getCurrentLink();
}
    

Link* BodySelectionManager::Impl::getCurrentLink()
{
    Link* link = nullptr;
    
    if(currentBodyItem){
        auto body = currentBodyItem->body();
        auto& selection = currentInfo->linkSelection;
        for(int i=0; i < selection.size(); ++i){
            if(selection[i] && i < body->numLinks()){
                link = body->link(i);
                break;
            }
        }
        if(!link){
            link = body->rootLink();
        }
    }

    return link;
}


void BodySelectionManager::setCurrent(BodyItem* bodyItem, Link* link)
{
    impl->setCurrentBodyItem(bodyItem, link);
}


void BodySelectionManager::Impl::setCurrentBodyItem(BodyItem* bodyItem, Link* link)
{
    bool bodyChanged = bodyItem != currentBodyItem;
    bool linkChanged = link && (link != getCurrentLink());

    if(bodyChanged || linkChanged){
        currentBodyItem = bodyItem;
        Link* currentLink;
        if(!bodyItem){
            currentInfo = nullptr;
            currentLink = nullptr;
        } else {
            currentInfo = getOrCreateBodyItemInfo(bodyItem, link);
            currentLink = getCurrentLink();
        }
        if(bodyChanged){
            sigCurrentBodyChanged(bodyItem);
        }
        sigCurrentBodySpecified(bodyItem);

        if(linkChanged){
            if(currentInfo){
                currentInfo->sigLinkSelectionChanged(currentInfo->linkSelection);
            }
        }
        sigCurrentChanged(bodyItem, currentLink);
        sigCurrentSpecified(bodyItem, currentLink);

    } else {
        sigCurrentBodySpecified(bodyItem);
        sigCurrentSpecified(bodyItem, getCurrentLink());
    }
}


BodyItemInfo* BodySelectionManager::Impl::getOrCreateBodyItemInfo(BodyItem* bodyItem, Link* link)
{
    BodyItemInfo* info = nullptr;
    auto body = bodyItem->body();
    auto iter = bodyItemInfoMap.find(bodyItem);
    
    if(iter != bodyItemInfoMap.end()){
        info = iter->second;
    } else {
        info = new BodyItemInfo;
        auto& selection = info->linkSelection;
        selection.resize(body->numLinks());
        if(!link){
            link = body->findUniqueEndLink();
            if(!link){
                link = body->rootLink();
            }
        }

        info->itemDisconnectConnection =
            bodyItem->sigDisconnectedFromRoot().connect(
                [this, bodyItem](){ onBodyItemDisconnectedFromRoot(bodyItem); });
        
        bodyItemInfoMap[bodyItem] = info;
    }

    auto& selection = info->linkSelection;
    selection.resize(body->numLinks());
    if(link){
        selection.assign(selection.size(), false);
        int linkIndex = link->index();
        if(linkIndex < body->numLinks()){
            selection[linkIndex] = true;
        }
    }

    currentInfo = info;
    
    return info;
}


void BodySelectionManager::Impl::onBodyItemDisconnectedFromRoot(BodyItem* bodyItem)
{
    auto iter = bodyItemInfoMap.find(bodyItem);
    if(iter != bodyItemInfoMap.end()){
        if(bodyItem == currentBodyItem){
            setCurrentBodyItem(nullptr, nullptr);
        }
        bodyItemInfoMap.erase(iter);
    }
}


SignalProxy<void(const ItemList<BodyItem>& selected)> BodySelectionManager::sigSelectedBodyItemsChanged()
{
    return impl->targetBodyItemPicker.sigSelectedItemsChanged();
}


const ItemList<BodyItem>& BodySelectionManager::selectedBodyItems() const
{
    return impl->targetBodyItemPicker.selectedItems();    
}


SignalProxy<void(const std::vector<bool>& selection)> BodySelectionManager::sigLinkSelectionChanged(BodyItem* bodyItem)
{
    return impl->getOrCreateBodyItemInfo(bodyItem, nullptr)->sigLinkSelectionChanged;
}


const std::vector<bool>& BodySelectionManager::linkSelection(BodyItem* bodyItem)
{
    return impl->getOrCreateBodyItemInfo(bodyItem, nullptr)->linkSelection;
}


void BodySelectionManager::setLinkSelection(BodyItem* bodyItem, const std::vector<bool>& linkSelection)
{
    auto info = impl->getOrCreateBodyItemInfo(bodyItem, nullptr);
    auto oldCurrentLink = impl->getCurrentLink();
    info->linkSelection = linkSelection;
    info->linkSelection.resize(bodyItem->body()->numLinks());
    info->sigLinkSelectionChanged(info->linkSelection);
    if(info == impl->currentInfo){
        auto currentLink = impl->getCurrentLink();
        if(currentLink != oldCurrentLink){
            impl->sigCurrentChanged(bodyItem, currentLink);
            impl->sigCurrentSpecified(bodyItem, currentLink);
        }
    }
}


bool BodySelectionManager::Impl::storeState(Archive& archive)
{
    if(currentBodyItem){
        archive.writeItemId("currentBodyItem", currentBodyItem);
        archive.write("currentLink", getCurrentLink()->name(), DOUBLE_QUOTED);
        return true;
    }
    return false;
}


void BodySelectionManager::Impl::restoreState(const Archive& archive)
{
    archive.addPostProcess(
        [&](){
            auto bodyItem = archive.findItem<BodyItem>("currentBodyItem");
            if(bodyItem){
                Link* link = nullptr;
                string linkName;
                if(archive.read("currentLink", linkName)){
                    link = bodyItem->body()->link(linkName);
                }
                setCurrentBodyItem(bodyItem, link);
            }
        });
}
