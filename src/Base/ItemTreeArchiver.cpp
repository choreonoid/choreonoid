/**
   @author Shin'ichiro Nakaoka
*/

#include "ItemTreeArchiver.h"
#include "RootItem.h"
#include "SubProjectItem.h"
#include "ItemManager.h"
#include "PluginManager.h"
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
    ostream& os;
    int itemIdCounter;
    int numArchivedItems;
    int numRestoredItems;
    const std::set<std::string>* pOptionalPlugins;

    Impl();
    ArchivePtr store(Archive& parentArchive, Item* item);
    ArchivePtr storeIter(Archive& parentArchive, Item* item, bool& isComplete);
    ItemList<> restore(Archive& archive, Item* parentItem, const std::set<std::string>& optionalPlugins);
    void restoreItemIter(Archive& archive, Item* parentItem, ItemList<>& restoredItems);
    ItemPtr restoreItem(
        Archive& archive, Item* parentItem, ItemList<>& restoredItems, string& out_itemName, bool& io_isOptional);
    void restoreItemStates(Archive& archive, Item* item);
};

}


ItemTreeArchiver::ItemTreeArchiver()
{
    impl = new Impl;
    reset();
}


ItemTreeArchiver::Impl::Impl()
    : mv(MessageView::instance()),
      os(mv->cout())
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
        mv->putln(_("Not all items were stored correctly."), MessageView::WARNING);
    }
    return archive;
}


ArchivePtr ItemTreeArchiver::Impl::storeIter(Archive& parentArchive, Item* item, bool& isComplete)
{
    string pluginName;
    string className;
    
    if(!ItemManager::getClassIdentifier(item, pluginName, className)){
        mv->putln(
            format(_("\"{}\" cannot be stored. Its type is not registered."), item->name()),
            MessageView::ERROR);
        isComplete = false;
        return nullptr;
    }

    ArchivePtr archive = new Archive();
    archive->inheritSharedInfoFrom(parentArchive);

    ArchivePtr dataArchive;

    if(!item->isSubItem()){
        mv->putln(format(_("Storing {0} \"{1}\""), className, item->name()));
        mv->flush();

        dataArchive = new Archive();
        dataArchive->inheritSharedInfoFrom(parentArchive);

        if(!item->store(*dataArchive)){
            mv->putln(format(_("\"{}\" cannot be stored."), item->name()), MessageView::ERROR);
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
    }
    if(item->isSelected()){
        archive->write("isSelected", true);
    }
    if(item->isChecked()){
        archive->write("isChecked", true);
    }
    if(!item->isSubItem()){
        if(!dataArchive->empty()){
            archive->insert("data", dataArchive);
        }
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
    ItemList<> restoredItems;

    archive.setCurrentParentItem(0);
    try {
        restoreItemIter(archive, parentItem, restoredItems);
    } catch (const ValueNode::Exception& ex){
        mv->putln(ex.message(), MessageView::ERROR);
    }
    archive.setCurrentParentItem(0);

    numRestoredItems = restoredItems.size();
    return restoredItems;
}


void ItemTreeArchiver::Impl::restoreItemIter(Archive& archive, Item* parentItem, ItemList<>& restoredItems)
{
    ItemPtr item;
    string itemName;
    bool isOptional = false;

    try {
        item = restoreItem(archive, parentItem, restoredItems, itemName, isOptional);
    } catch (const ValueNode::Exception& ex){
        mv->putln(ex.message(), MessageView::ERROR);
    }
    
    if(!item){
        if(!isOptional){
            if(itemName.empty()){
                mv->putln(_("Item cannot be restored."), MessageView::ERROR);
            } else {
                mv->putln(format(_("\"{}\" cannot be restored."), itemName), MessageView::ERROR);
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
                restoreItemIter(*childArchive, item, restoredItems);
            }
        }
    }
}


ItemPtr ItemTreeArchiver::Impl::restoreItem
(Archive& archive, Item* parentItem, ItemList<>& restoredItems, string& out_itemName, bool& io_isOptional)
{
    ItemPtr item;
    string& name = out_itemName;

    if(!archive.read("name", name)){
        return item;
    }

    const bool isSubItem = archive.get("isSubItem", false);
    if(isSubItem){
        item = parentItem->findSubItem(name);
        if(!item){
            mv->putln(
                format(_("Sub item \"{}\" is not found. Its children cannot be restored."), name),
                MessageView::ERROR);
        }
        restoreItemStates(archive, item);
        return item;
    }
    
    string pluginName;
    string className;
    if(!(archive.read("plugin", pluginName) && archive.read("class", className))){
        mv->putln(_("Archive is broken."), MessageView::ERROR);
        return item;
    }

    const char* actualPluginName = PluginManager::instance()->guessActualPluginName(pluginName);
    if(actualPluginName){
        item = ItemManager::createItem(actualPluginName, className);
    } else {
        io_isOptional = (pOptionalPlugins->find(pluginName) != pOptionalPlugins->end());
        if(!io_isOptional){
            mv->putln(format(_("{}Plugin is not loaded."), pluginName), MessageView::ERROR);
        }
    }

    if(!item){
        if(!io_isOptional){
            mv->putln(
                format(_("{0} of {1}Plugin is not a registered item type."), className, pluginName),
                MessageView::ERROR);
            ++numArchivedItems;
        }
        return item;
    }

    ++numArchivedItems;
        
    item->setName(name);
    bool isRootItem = dynamic_pointer_cast<RootItem>(item);
    if(isRootItem){
        item = parentItem;
        --numArchivedItems;
    } else {
        mv->putln(format(_("Restoring {0} \"{1}\""), className, name));
        mv->flush();

        ValueNodePtr dataNode = archive.find("data");
        if(dataNode->isValid()){
            if(!dataNode->isMapping()){
                mv->putln(_("The 'data' key does not have mapping-type data"), MessageView::ERROR);
                item.reset();
            } else {
                Archive* dataArchive = static_cast<Archive*>(dataNode->toMapping());
                dataArchive->inheritSharedInfoFrom(archive);
                dataArchive->setCurrentParentItem(parentItem);
                if(!item->restore(*dataArchive)){
                    item.reset();
                }
            }
        }
        if(item){
            parentItem->addChildItem(item);
            restoreItemStates(archive, item);
            restoredItems.push_back(item);
        }
    }

    return item;
}


void ItemTreeArchiver::Impl::restoreItemStates(Archive& archive, Item* item)
{
    if(archive.get("isSelected", false)){
        item->setSelected(true);
    }
    if(archive.get("isChecked", false)){
        item->setChecked(true);
    }
}
