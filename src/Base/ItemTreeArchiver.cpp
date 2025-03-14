#include "ItemTreeArchiver.h"
#include "ItemAddon.h"
#include "RootItem.h"
#include "SubProjectItem.h"
#include "ItemManager.h"
#include "Archive.h"
#include <cnoid/MessageOut>
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <cnoid/Format>
#include <list>
#include <set>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class ItemTreeArchiver::Impl
{
public:
    int itemIdCounter;
    int numArchivedItems;
    int numRestoredItems;
    const std::set<std::string>* pOptionalPlugins;
    bool isTemporaryItemSaveEnabled;
    MessageOut* mout;

    Impl();
    ArchivePtr store(Archive& parentArchive, Item* item);
    void registerItemIdIter(Archive& topArchive, Item* item);
    ArchivePtr storeIter(Archive& parentArchive, Item* item, bool& isComplete);
    bool checkIfTemporary(Item* item);
    bool checkSubTreeTemporality(Item* item);
    void storeAddons(Archive& archive, Item* item);
    ItemList<> restore(Archive& archive, Item* parentItem, const std::set<std::string>& optionalPlugins);
    void restoreItemIter(Archive& archive, Item* parentItem, ItemList<>& io_topLevelItems, int level);
    ItemPtr restoreItem(
        Archive& archive, Item* parentItem, string& itemName, string& classame,
        bool& io_isRootItem, bool& io_isOptional);
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
{
    mout = MessageOut::master();
}


ItemTreeArchiver::~ItemTreeArchiver()
{
    delete impl;
}


void ItemTreeArchiver::setMessageOut(MessageOut* mout)
{
    impl->mout = mout;
}


void ItemTreeArchiver::reset()
{
    impl->itemIdCounter = 0;
    impl->numArchivedItems = 0;
    impl->numRestoredItems = 0;
    impl->pOptionalPlugins = nullptr;
    impl->isTemporaryItemSaveEnabled = false;
}


void ItemTreeArchiver::setTemporaryItemSaveEnabled(bool on)
{
    impl->isTemporaryItemSaveEnabled = on;
}


bool ItemTreeArchiver::isTemporaryItemSaveEnabled() const
{
    return impl->isTemporaryItemSaveEnabled;
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
    registerItemIdIter(parentArchive, item);

    bool isComplete = true;
    ArchivePtr archive = storeIter(parentArchive, item, isComplete);
    if(!isComplete){
        mout->putWarningln(_("Not all items were stored correctly."));
    }
    
    return archive;
}


void ItemTreeArchiver::Impl::registerItemIdIter(Archive& topArchive, Item* item)
{
    if(!item->isSubItem()){
        topArchive.registerItemId(item, itemIdCounter++);
    }
    for(auto childItem = item->childItem(); childItem; childItem = childItem->nextItem()){
        registerItemIdIter(topArchive, childItem);
    }
}
    

ArchivePtr ItemTreeArchiver::Impl::storeIter(Archive& parentArchive, Item* item, bool& isComplete)
{
    string pluginName;
    string className;
    
    if(!ItemManager::getClassIdentifier(item, pluginName, className)){
        mout->putErrorln(
            formatR(_("\"{}\" cannot be stored. Its type is not registered."), item->displayName()));
        isComplete = false;
        return nullptr;
    }

    ArchivePtr archive = new Archive;
    archive->inheritSharedInfoFrom(parentArchive);

    ArchivePtr dataArchive;

    if(!item->isSubItem()){
        mout->putln(formatR(_("Storing {0} \"{1}\""), className, item->displayName()));

        dataArchive = new Archive;
        dataArchive->inheritSharedInfoFrom(parentArchive);

        if(!item->store(*dataArchive)){
            mout->putErrorln(formatR(_("\"{}\" cannot be stored."), item->displayName()));
            isComplete = false;
            return nullptr;
        }

        archive->insert("id", archive->getItemIdNode(item));
    }

    archive->write("name", item->name(), DOUBLE_QUOTED);

    auto subProjectItem = dynamic_cast<SubProjectItem*>(item);
    if(subProjectItem && subProjectItem->isSavingSubProject()){
        pluginName = "Base";
        className = "RootItem";
    }

    if(item->isSubItem()){
        archive->write("is_sub_item", true);
    } else {
        archive->write("plugin", pluginName);
        archive->write("class", className);
        if(item->hasAttribute(Item::Builtin)){
            archive->write("is_builtin_item", true);
        }
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

    if(!archive->isSavingProjectAsBackup()){
        item->setConsistentWithProjectArchive(true);
    }

    if(subProjectItem && !subProjectItem->isSavingSubProject()){
        return archive;
    }

    ListingPtr children = new Listing;
    for(auto childItem = item->childItem(); childItem; childItem = childItem->nextItem()){
        if(checkIfTemporary(childItem)){
            continue;
        }
        if(!isTemporaryItemSaveEnabled && childItem->isTemporary()){
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


bool ItemTreeArchiver::Impl::checkIfTemporary(Item* item)
{
    bool isTemporary = false;
    if(item->isTemporary()){
        if(isTemporaryItemSaveEnabled || !checkSubTreeTemporality(item)){
            item->setTemporary(false);
            item->notifyUpdate();
        } else {
            isTemporary = true;
        }
    }
    return isTemporary;
}


bool ItemTreeArchiver::Impl::checkSubTreeTemporality(Item* item)
{
    for(auto childItem = item->childItem(); childItem; childItem = childItem->nextItem()){
        if(childItem->isSubItem()){
            continue;
        }
        if(!childItem->isTemporary()){
            return false;
        }
        if(!checkSubTreeTemporality(childItem)){
            return false;
        }
    }
    return true;
}


void ItemTreeArchiver::Impl::storeAddons(Archive& archive, Item* item)
{
    auto addons = item->addons();
    if(!addons.empty()){
        ListingPtr addonList = new Listing;
        for(auto& addon : addons){
            string name, moduleName;
            if(!ItemManager::getAddonIdentifier(addon, moduleName, name)){
                mout->putErrorln(
                    formatR(_("Addon \"{0}\" of item \"{1}\" cannot be stored. Its type is not registered."),
                            typeid(*addon).name(), item->displayName()));
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
                    mout->putErrorln(formatR(_("Addon \"{0}\" of item \"{1}\" cannot be stored."),
                                      name, item->name()));
                    */
                }
            }
        }
        if(!addonList->empty()){
            archive.insert("addons", addonList);
        }
    }
}


ItemList<> ItemTreeArchiver::restore(Archive* archive, Item* parentItem, const std::set<std::string>& optionalPlugins)
{
    return impl->restore(*archive, parentItem, optionalPlugins);
}


ItemList<> ItemTreeArchiver::Impl::restore
(Archive& archive, Item* parentItem, const std::set<std::string>& optionalPlugins)
{
    numArchivedItems = 0;
    numRestoredItems = 0;
    pOptionalPlugins = &optionalPlugins;
    ItemList<> topLevelItems;

    archive.setCurrentParentItem(nullptr);
    try {
        restoreItemIter(archive, parentItem, topLevelItems, 0);
    } catch (const ValueNode::Exception& ex){
        mout->putErrorln(ex.message());
    }
    archive.setCurrentParentItem(nullptr);

    return topLevelItems;
}


void ItemTreeArchiver::Impl::restoreItemIter
(Archive& archive, Item* parentItem, ItemList<>& io_topLevelItems, int level)
{
    ItemPtr item;
    string itemName;
    string className;
    bool isRootItem = false;
    bool isOptional = false;

    try {
        item = restoreItem(archive, parentItem, itemName, className, isRootItem, isOptional);
    } catch (const ValueNode::Exception& ex){
        mout->putErrorln(ex.message());
    }
    
    if(!item){
        if(!isOptional){
            if(!itemName.empty()){
                if(!className.empty()){
                    mout->putErrorln(formatR(_("{0} \"{1}\" cannot be restored."), className, itemName));
                } else {
                    mout->putErrorln(formatR(_("\"{0}\" cannot be restored."), itemName));
                }
            } else {
                if(!className.empty()){
                    mout->putErrorln(formatR(_("An instance of {0} cannot be restored."), className));
                } else {
                    mout->putErrorln(_("An instance of unkown item type cannot be restored."));
                }                    
            }
        }
    } else {
        int id;
        if(archive.read("id", id) && (id >= 0)){
            archive.registerItemId(item, id);
        }
        if(!isRootItem){
            if(level == 0){
                io_topLevelItems.push_back(item);
            }
            ++level;
        }
        ListingPtr children = archive.findListing("children");
        if(children->isValid()){
            for(int i=0; i < children->size(); ++i){
                Archive* childArchive = dynamic_cast<Archive*>(children->at(i)->toMapping());
                childArchive->inheritSharedInfoFrom(archive);
                restoreItemIter(*childArchive, item, io_topLevelItems, level);
            }
        }

        archive.setCurrentItem(item);
        archive.callProcessesOnSubTreeRestored(item);
    }
}


ItemPtr ItemTreeArchiver::Impl::restoreItem
(Archive& archive, Item* parentItem, string& itemName, string& className, bool& io_isRootItem, bool& io_isOptional)
{
    archive.read("name", itemName);

    const bool isSubItem = archive.get({ "is_sub_item", "isSubItem" }, false);
    if(isSubItem){
        if(itemName.empty()){
            mout->putErrorln(_("The archive has an empty-name sub item, which cannot be processed."));
            return nullptr;
        }
        ItemPtr subItem = parentItem->findChildItem(itemName, [](Item* item){ return item->isSubItem(); });
        if(!subItem){
            mout->putErrorln(
                formatR(_("Sub item \"{}\" is not found. Its children cannot be restored."), itemName));
        }
        restoreItemStates(archive, subItem);
        return subItem;
    }
    
    string pluginName;
    if(!(archive.read("plugin", pluginName) && archive.read("class", className))){
        mout->putErrorln(_("Archive is broken."));
        return nullptr;
    }

    ItemPtr item = ItemManager::createItem(pluginName, className);
    if(!item){
        io_isOptional = (pOptionalPlugins->find(pluginName) != pOptionalPlugins->end());
        if(!io_isOptional){
            mout->putErrorln(
                formatR(_("{0} of {1}Plugin is not a registered item type."), className, pluginName));
            ++numArchivedItems;
        }
        return nullptr;
    }

    archive.setCurrentItem(item);

    ++numArchivedItems;
        
    io_isRootItem = bool(dynamic_pointer_cast<RootItem>(item));
    if(io_isRootItem){
        item = parentItem;
        --numArchivedItems;

    } else {
        item->setName(itemName);

        if(archive.get("is_builtin_item", false)){
            item->setAttribute(Item::Builtin);
        }
        if(archive.get("is_attached_item", false)){
            item->setAttribute(Item::Attached);
        }
        
        mout->putln(formatR(_("Restoring {0} \"{1}\""), className, itemName));

        ValueNodePtr dataNode = archive.find("data");
        ArchivePtr dataArchive;
        if(dataNode->isValid()){
            if(!dataNode->isMapping()){
                mout->putErrorln(_("The 'data' key does not have mapping-type data."));
                item.reset();
            } else {
                dataArchive = static_cast<Archive*>(dataNode->toMapping());
            }
        }
        if(item){
            if(!dataArchive){
                dataArchive = new Archive;
            }
            dataArchive->inheritSharedInfoFrom(archive);
            dataArchive->setCurrentParentItem(parentItem);
            if(!item->restore(*dataArchive)){
                item.reset();
            } else {
                restoreAddons(archive, item);
            }
        }
        if(item){
            if(!parentItem->addChildItem(item)){
                mout->putErrorln(
                    formatR(_("{0} \"{1}\" cannot be added to \"{2}\" as a child item."),
                            className, itemName, parentItem->displayName()));
                item.reset();
            } else {
                restoreItemStates(archive, item);
                ++numRestoredItems;
            }
        }
    }

    return item;
}


void ItemTreeArchiver::Impl::restoreAddons(Archive& archive, Item* item)
{
    auto addonsNode = archive.find("addons");
    if(addonsNode->isValid()){
        if(!addonsNode->isListing()){
            mout->putErrorln(_("The 'addons' value must be a listing."));
        } else {
            string name;
            string moduleName;
            auto addonList = addonsNode->toListing();
            for(int i=0; i < addonList->size(); ++i){
                auto addonArchive = dynamic_cast<Archive*>(addonList->at(i)->toMapping());
                addonArchive->inheritSharedInfoFrom(archive);
                if(!(addonArchive->read("name", name) && addonArchive->read("plugin", moduleName))){
                    mout->putErrorln(
                        formatR(_("The name and plugin are not specified at addon {0}."), i));
                } else {
                    ItemAddonPtr addon = ItemManager::createAddon(moduleName, name);
                    if(!addon){
                        mout->putErrorln(
                            formatR(_("Addon \"{0}\" of plugin \"{1}\" cannot be created."),
                                    name, moduleName));
                    } else {
                        if(!item->setAddon(addon)){
                            mout->putErrorln(
                                formatR(_("Addon \"{0}\" cannot be added to item \"{1}\"."),
                                        name, item->displayName()));
                        } else {
                            if(!addon->restore(*addonArchive)){
                                mout->putErrorln(
                                    formatR(_("Addon \"{0}\" of plugin \"{1}\" cannot be restored."),
                                            name, moduleName));
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
