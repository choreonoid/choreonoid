/**
   @author Shin'ichiro Nakaoka
*/

#include "ItemTreeArchiver.h"
#include "RootItem.h"
#include "ItemManager.h"
#include "PluginManager.h"
#include "MessageView.h"
#include "Archive.h"
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <set>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace cnoid {

class ItemTreeArchiverImpl
{
public:
    MessageView* mv;
    ostream& os;
    int itemIdCounter;
    int numArchivedItems;
    int numRestoredItems;
    std::set<string> noExistingPluginNames;

    ItemTreeArchiverImpl();
    ArchivePtr store(Archive& parentArchive, Item* item);
    ArchivePtr storeIter(Archive& parentArchive, Item* item, bool& isComplete);
    bool restore(Archive& archive, Item* parentItem);
    void restoreItemIter(Archive& archive, Item* parentItem);
};

}


ItemTreeArchiver::ItemTreeArchiver()
{
    impl = new ItemTreeArchiverImpl;
    reset();
}


ItemTreeArchiverImpl::ItemTreeArchiverImpl()
    : mv(MessageView::mainInstance()),
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
    impl->noExistingPluginNames.clear();
}


int ItemTreeArchiver::numArchivedItems() const
{
    return impl->numArchivedItems;
}


int ItemTreeArchiver::numRestoredItems() const
{
    return impl->numRestoredItems;
}


ArchivePtr ItemTreeArchiver::store(ArchivePtr parentArchive, Item* item)
{
    return impl->store(*parentArchive, item);
}


ArchivePtr ItemTreeArchiverImpl::store(Archive& parentArchive, Item* item)
{
    bool isComplete = true;
    ArchivePtr archive = storeIter(parentArchive, item, isComplete);
    if(!isComplete){
        mv->putln(MessageView::WARNING, _("Not all items were stored correctly."));
    }
    return archive;
}


ArchivePtr ItemTreeArchiverImpl::storeIter(Archive& parentArchive, Item* item, bool& isComplete)
{
    string pluginName;
    string className;
    
    if(!ItemManager::getClassIdentifier(item, pluginName, className)){
        mv->putln(MessageView::ERROR,
                  format(_("\"%1%\" cannot be stored. Its type is not registered.")) % item->name());
        isComplete = false;
        return 0;
    }

    ArchivePtr archive = new Archive();
    archive->inheritSharedInfoFrom(parentArchive);

    ArchivePtr dataArchive;

    if(!item->isSubItem()){
        mv->putln(format(_("Storing %1% \"%2%\"")) % className % item->name());
        mv->flush();

        dataArchive = new Archive();
        dataArchive->inheritSharedInfoFrom(parentArchive);

        if(!item->store(*dataArchive)){
            mv->putln(MessageView::ERROR, format(_("\"%1%\" cannot be stored.")) % item->name());
            isComplete = false;
            return 0;
        }

        archive->registerItemId(item, itemIdCounter);
        archive->write("id", itemIdCounter);
        itemIdCounter++;
    }

    archive->write("name", item->name(), DOUBLE_QUOTED);

    if(item->isSubItem()){
        archive->write("isSubItem", true);
    } else {
        archive->write("plugin", pluginName);
        archive->write("class", className);
        if(!dataArchive->empty()){
            archive->insert("data", dataArchive);
        }
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
        archive = 0;
    }

    return archive;
}


bool ItemTreeArchiver::restore(ArchivePtr archive, Item* parentItem)
{
    return impl->restore(*archive, parentItem);
}


bool ItemTreeArchiverImpl::restore(Archive& archive, Item* parentItem)
{
    numArchivedItems = 0;
    numRestoredItems = 0;

    archive.setCurrentParentItem(0);
    try {
        restoreItemIter(archive, parentItem);
    } catch (const ValueNode::Exception& ex){
        os << ex.message();
    }
    archive.setCurrentParentItem(0);
    noExistingPluginNames.clear();

    return (numRestoredItems > 0);
}


void ItemTreeArchiverImpl::restoreItemIter(Archive& archive, Item* parentItem)
{
    string name;
    if(!archive.read("name", name)){
        return;
    }

    ItemPtr item;

    const bool isSubItem = archive.get("isSubItem", false);
    if(isSubItem){
        item = parentItem->findSubItem(name);
        if(!item){
            mv->putln(MessageView::WARNING,
                      format(_("Sub item \"%1%\" is not found. Its children cannot be restored.")) % name);
        }
    } else {
        ++numArchivedItems;
        string pluginName;
        string className;
        if(!(archive.read("plugin", pluginName) && archive.read("class", className))){
            mv->putln(MessageView::ERROR, _("Archive is broken."));
            return;
        }
        const char* actualPluginName = PluginManager::instance()->guessActualPluginName(pluginName);
        if(actualPluginName){
            item = ItemManager::create(actualPluginName, className);
        } else {
            if(noExistingPluginNames.insert(pluginName).second){
                mv->putln(MessageView::WARNING, format(_("%1%Plugin is not loaded.")) % pluginName);
            }
        }
        if(!item){
            mv->putln(MessageView::WARNING,
                      format(_("%1% of %2%Plugin is not a registered item type."))
                      % className % pluginName);
        } else {
            item->setName(name);
            bool isRootItem = dynamic_pointer_cast<RootItem>(item);
            if(isRootItem){
                item = parentItem;
                --numArchivedItems;
            } else {
                mv->putln(format(_("Restoring %1% \"%2%\"")) % className % name);
                mv->flush();
                
                ValueNodePtr dataNode = archive.find("data");
                if(dataNode->isValid()){
                    if(!dataNode->isMapping()){
                        mv->putln(MessageView::ERROR, _("The 'data' key does not have mapping-type data"));
                        item = 0;
                    } else {
                        Archive* dataArchive = static_cast<Archive*>(dataNode->toMapping());
                        dataArchive->inheritSharedInfoFrom(archive);
                        dataArchive->setCurrentParentItem(parentItem);
                        if(!item->restore(*dataArchive)){
                            item = 0;
                        }
                    }
                }
                if(item){
                    parentItem->addChildItem(item);
                    ++numRestoredItems;
                }
            }
        }
    }

    if(!item){
        mv->putln(MessageView::WARNING, format(_("\"%1%\" cannot be restored.")) % name);
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
    }
}
