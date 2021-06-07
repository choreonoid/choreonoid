/**
   @author Shin'ichiro Nakaoka
*/

#include "ItemTreeArchiver.h"
#include "ItemAddon.h"
#include "RootItem.h"
#include "SubProjectItem.h"
#include "ItemManager.h"
#include "MessageView.h"
#include "Archive.h"
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <set>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class ItemTreeArchiver::Impl
{
public:
    MessageView* mv;
    int itemIdCounter;
    int numArchivedItems;
    int numRestoredItems;
    const std::set<std::string>* pOptionalPlugins;

    Impl();
    ArchivePtr store(Archive& parentArchive, Item* item);
    ArchivePtr storeIter(Archive& parentArchive, Item* item, bool& isComplete);
    void storeAddons(Archive& archive, Item* item);
    void restore(Archive& archive, Item* parentItem, const std::set<std::string>& optionalPlugins);
    void restoreItemIter(Archive& archive, Item* parentItem);
    ItemPtr restoreItem(
        Archive& archive, Item* parentItem, string& itemName, string& classame, bool& io_isOptional);
    void restoreAddons(Archive& archive, Item* item);
    void restoreItemStates(Archive& archive, Item* item);
};

}


ItemTreeArchiver::ItemTreeArchiver()
{
    impl = new Impl;
    reset();
}


ItemTreeArchiver::Impl::Impl()
    : mv(MessageView::instance())
{
    
}


ItemTreeArchiver::~ItemTreeArchiver()
{
    delete impl;
}


void ItemTreeArchiver::reset()
{
    impl->itemIdCounter = 0;
    impl->numArchivedItems = 0;
    impl->numRestoredItems = 0;
    impl->pOptionalPlugins = nullptr;
}


int ItemTreeArchiver::numArchivedItems() const
{
    return impl->numArchivedItems;
}


int ItemTreeArchiver::numRestoredItems() const
{
    return impl->numRestoredItems;
}


ArchivePtr ItemTreeArchiver::store(Archive* parentArchive, Item* item)
{
    return impl->store(*parentArchive, item);
}


ArchivePtr ItemTreeArchiver::Impl::store(Archive& parentArchive, Item* item)
{
    bool isComplete = true;
    ArchivePtr archive = storeIter(parentArchive, item, isComplete);
    if(!isComplete){
        mv->putln(_("Not all items were stored correctly."), MessageView::Warning);
    }
    return archive;
}


ArchivePtr ItemTreeArchiver::Impl::storeIter(Archive& parentArchive, Item* item, bool& isComplete)
{
    string pluginName;
    string className;
    
    if(!ItemManager::getClassIdentifier(item, pluginName, className)){
        mv->putln(
            format(_("\"{}\" cannot be stored. Its type is not registered."), item->displayName()),
            MessageView::Error);
        isComplete = false;
        return nullptr;
    }

    ArchivePtr archive = new Archive;
    archive->inheritSharedInfoFrom(parentArchive);

    ArchivePtr dataArchive;

    if(!item->isSubItem()){
        mv->putln(format(_("Storing {0} \"{1}\""), className, item->displayName()));
        mv->flush();

        dataArchive = new Archive;
        dataArchive->inheritSharedInfoFrom(parentArchive);

        if(!item->store(*dataArchive)){
            mv->putln(format(_("\"{}\" cannot be stored."), item->displayName()), MessageView::Error);
            isComplete = false;
            return nullptr;
        }

        archive->registerItemId(item, itemIdCounter);
        archive->write("id", itemIdCounter);
        itemIdCounter++;
    }

    archive->write("name", item->name(), DOUBLE_QUOTED);

    auto subProjectItem = dynamic_cast<SubProjectItem*>(item);
    if(subProjectItem && subProjectItem->isSavingSubProject()){
        pluginName = "Base";
        className = "RootItem";
    }

    if(item->isSubItem()){
        archive->write("isSubItem", true);
    } else {
        archive->write("plugin", pluginName);
        archive->write("class", className);
        if(item->hasAttribute(Item::Attached)){
            archive->write("is_attached_item", true);
        }
    }
    if(item->isSelected()){
        archive->write("is_selected", true);
    }
    if(item->isChecked()){
        archive->write("is_checked", true);
    }
    if(!item->isSubItem()){
        if(!dataArchive->empty()){
            archive->insert("data", dataArchive);
        }
        storeAddons(*archive, item);
    }

    if(subProjectItem && !subProjectItem->isSavingSubProject()){
        return archive;
    }

    ListingPtr children = new Listing();
    for(Item* childItem = item->childItem(); childItem; childItem = childItem->nextItem()){
        if(childItem->isTemporal()){
            continue;
        }
        ArchivePtr childArchive = storeIter(*archive, childItem, isComplete);
        if(childArchive){
            children->append(childArchive);
        }
    }
    if(!children->empty()){
        archive->insert("children", children);
    } else if(item->isSubItem()){
        // Sub item without childiren is not stored
        archive = nullptr;
    }

    return archive;
}


void ItemTreeArchiver::Impl::storeAddons(Archive& archive, Item* item)
{
    auto addons = item->addons();
    if(!addons.empty()){
        ListingPtr addonList = new Listing;
        for(auto& addon : addons){
            string name, moduleName;
            if(!ItemManager::getAddonIdentifier(addon, moduleName, name)){
                mv->putln(
                    format(_("Addon \"{0}\" of item \"{1}\" cannot be stored. Its type is not registered."),
                           typeid(*addon).name(), item->displayName()),
                    MessageView::Error);
            } else {
                ArchivePtr addonArchive = new Archive;
                addonArchive->inheritSharedInfoFrom(archive);
                addonArchive->write("name", name);
                addonArchive->write("plugin", moduleName);
                if(addon->store(*addonArchive)){
                    addonList->append(addonArchive);
                } else {
                    //! \note Storing the addon data is just skipped when the store function returns false.
                    /*
                    mv->putln(format(_("Addon \"{0}\" of item \"{1}\" cannot be stored."),
                                     name, item->name()),
                              MessageView::Error);
                    */
                }
            }
        }
        if(!addonList->empty()){
            archive.insert("addons", addonList);
        }
    }
}


void ItemTreeArchiver::restore(Archive* archive, Item* parentItem, const std::set<std::string>& optionalPlugins)
{
    impl->restore(*archive, parentItem, optionalPlugins);
}


void ItemTreeArchiver::Impl::restore
(Archive& archive, Item* parentItem, const std::set<std::string>& optionalPlugins)
{
    numArchivedItems = 0;
    numRestoredItems = 0;
    pOptionalPlugins = &optionalPlugins;

    archive.setCurrentParentItem(nullptr);
    try {
        restoreItemIter(archive, parentItem);
    } catch (const ValueNode::Exception& ex){
        mv->putln(ex.message(), MessageView::Error);
    }
    archive.setCurrentParentItem(nullptr);
}


void ItemTreeArchiver::Impl::restoreItemIter(Archive& archive, Item* parentItem)
{
    ItemPtr item;
    string itemName;
    string className;
    bool isOptional = false;
    std::vector<std::function<void()>> processesOnSubTreeRestored;
    archive.setPointerToProcessesOnSubTreeRestored(&processesOnSubTreeRestored);

    try {
        item = restoreItem(archive, parentItem, itemName, className, isOptional);
    } catch (const ValueNode::Exception& ex){
        mv->putln(ex.message(), MessageView::Error);
    }
    
    if(!item){
        if(!isOptional){
            if(!itemName.empty()){
                if(!className.empty()){
                    mv->putln(format(_("{0} \"{1}\" cannot be restored."), className, itemName), MessageView::Error);
                } else {
                    mv->putln(format(_("\"{0}\" cannot be restored."), itemName), MessageView::Error);
                }
            } else {
                if(!className.empty()){
                    mv->putln(format(_("An instance of {0} cannot be restored."), className), MessageView::Error);
                } else {
                    mv->putln(_("An instance of unkown item type cannot be restored."), MessageView::Error);
                }                    
            }
        }
    } else {
        int id;
        if(archive.read("id", id) && (id >= 0)){
            archive.registerItemId(item, id);
        }
        ListingPtr children = archive.findListing("children");
        if(children->isValid()){
            for(int i=0; i < children->size(); ++i){
                Archive* childArchive = dynamic_cast<Archive*>(children->at(i)->toMapping());
                childArchive->inheritSharedInfoFrom(archive);
                restoreItemIter(*childArchive, item);
            }
        }
        for(auto& func : processesOnSubTreeRestored){
            func();
        }
    }
}


ItemPtr ItemTreeArchiver::Impl::restoreItem
(Archive& archive, Item* parentItem, string& itemName, string& className, bool& io_isOptional)
{
    archive.read("name", itemName);

    const bool isSubItem = archive.get("isSubItem", false);
    if(isSubItem){
        if(itemName.empty()){
            mv->putln(_("The archive has an empty-name sub item, which cannot be processed."), MessageView::Error);
            return nullptr;
        }
        ItemPtr subItem = parentItem->findChildItem(itemName, [](Item* item){ return item->isSubItem(); });
        if(!subItem){
            mv->putln(
                format(_("Sub item \"{}\" is not found. Its children cannot be restored."), itemName),
                MessageView::Error);
        }
        restoreItemStates(archive, subItem);
        return subItem;
    }
    
    string pluginName;
    if(!(archive.read("plugin", pluginName) && archive.read("class", className))){
        mv->putln(_("Archive is broken."), MessageView::Error);
        return nullptr;
    }

    ItemPtr item = ItemManager::createItem(pluginName, className);
    if(!item){
        io_isOptional = (pOptionalPlugins->find(pluginName) != pOptionalPlugins->end());
        if(!io_isOptional){
            mv->putln(
                format(_("{0} of {1}Plugin is not a registered item type."), className, pluginName),
                MessageView::Error);
            ++numArchivedItems;
        }
        return nullptr;
    }

    ++numArchivedItems;
        
    item->setName(itemName);
    bool isRootItem = dynamic_pointer_cast<RootItem>(item);
    if(isRootItem){
        item = parentItem;
        --numArchivedItems;

    } else {
        if(archive.get("is_attached_item", false)){
            item->setAttribute(Item::Attached);
        }
        
        mv->putln(format(_("Restoring {0} \"{1}\""), className, itemName));
        mv->flush();

        ValueNodePtr dataNode = archive.find("data");
        if(dataNode->isValid()){
            if(!dataNode->isMapping()){
                mv->putln(_("The 'data' key does not have mapping-type data."), MessageView::Error);
                item.reset();
            } else {
                Archive* dataArchive = static_cast<Archive*>(dataNode->toMapping());
                dataArchive->inheritSharedInfoFrom(archive);
                dataArchive->setCurrentParentItem(parentItem);
                if(!item->restore(*dataArchive)){
                    item.reset();
                } else {
                    restoreAddons(archive, item);
                }
            }
        }
        if(item){
            parentItem->addChildItem(item);
            restoreItemStates(archive, item);
            ++numRestoredItems;
        }
    }

    return item;
}


void ItemTreeArchiver::Impl::restoreAddons(Archive& archive, Item* item)
{
    auto addonsNode = archive.find("addons");
    if(addonsNode->isValid()){
        if(!addonsNode->isListing()){
            mv->putln(_("The 'addons' value must be a listing."), MessageView::Error);
        } else {
            string name;
            string moduleName;
            auto addonList = addonsNode->toListing();
            for(int i=0; i < addonList->size(); ++i){
                auto addonArchive = dynamic_cast<Archive*>(addonList->at(i)->toMapping());
                addonArchive->inheritSharedInfoFrom(archive);
                if(!(addonArchive->read("name", name) && addonArchive->read("plugin", moduleName))){
                    mv->putln(format(_("The name and plugin are not specified at addon {0}."), i),
                              MessageView::Error);
                } else {
                    ItemAddonPtr addon = ItemManager::createAddon(moduleName, name);
                    if(!addon){
                        mv->putln(format(_("Addon \"{0}\" of plugin \"{1}\" cannot be created."),
                                         name, moduleName), MessageView::Error);
                    } else {
                        if(!item->setAddon(addon)){
                            mv->putln(format(_("Addon \"{0}\" cannot be added to item \"{1}\"."),
                                             name, item->displayName()), MessageView::Error);
                        } else {
                            if(!addon->restore(*addonArchive)){
                                mv->putln(format(_("Addon \"{0}\" of plugin \"{1}\" cannot be restored."),
                                                 name, moduleName), MessageView::Error);
                                item->removeAddon(addon);
                            }
                        }
                    }
                }
            }
        }
    }
}


void ItemTreeArchiver::Impl::restoreItemStates(Archive& archive, Item* item)
{
    bool state;
    if(archive.read("is_selected", state) || archive.read("isSelected", state)){
        item->setSelected(state);
    }
    if(archive.read("is_checked", state) || archive.read("isChecked", state)){
        item->setChecked(state);
    }
}
