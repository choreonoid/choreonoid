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
#include <vector>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;


ItemTreeArchiver::ItemTreeArchiver()
{
    messageView = MessageView::mainInstance();
}


ItemTreeArchiver::~ItemTreeArchiver()
{

}


ArchivePtr ItemTreeArchiver::store(ArchivePtr parentArchive, Item* item)
{
    itemIdCounter = 0;
    return storeIter(*parentArchive, item);
}


ArchivePtr ItemTreeArchiver::storeIter(Archive& parentArchive, Item* item)
{
    string pluginName;
    string className;
    
    if(!ItemManager::getClassIdentifier(item, pluginName, className)){
        messageView->putln(fmt(_("\"%1%\" cannot be stored. Its type is not registered.")) % item->name());
        return 0;
    }

    ArchivePtr archive = new Archive();
    archive->inheritSharedInfoFrom(parentArchive);

    ArchivePtr dataArchive;

    if(!item->isSubItem()){
        messageView->putln(fmt(_("Storing %1% \"%2%\"")) % className % item->name());
        messageView->flush();

        dataArchive = new Archive();
        dataArchive->inheritSharedInfoFrom(parentArchive);

        if(!item->store(*dataArchive)){
            messageView->putln(fmt(_("\"%1%\" cannot be stored.")) % item->name());
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
        ArchivePtr childArchive = storeIter(*archive, childItem);
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
    bool result = false;

    archive->setCurrentParentItem(0);
    
    try {
        result = restoreItemIter(*archive, parentItem);
    } catch (const ValueNode::Exception& ex){
        messageView->put(ex.message());
    }

    archive->setCurrentParentItem(0);

    return result;
}


bool ItemTreeArchiver::restoreItemIter(Archive& archive, Item* parentItem)
{
    string name;
    if(!archive.read("name", name)){
        return false;
    }
    
    ItemPtr item;

    const bool isSubItem = archive.get("isSubItem", false);

    if(isSubItem){
        item = parentItem->findSubItem(name);
        if(!item){
            messageView->putln(fmt(_("Sub item \"%1%\" is not found. Its children cannot be restored.")) % name);
        }
    } else {
        string pluginName;
        string className;
        if(!(archive.read("plugin", pluginName) && archive.read("class", className))){
            messageView->putln(_("Archive is broken."));
            return false;
        }
        const char* actualPluginName = PluginManager::instance()->guessActualPluginName(pluginName);
        if(actualPluginName){
            item = ItemManager::create(actualPluginName, className);
        }

        if(!item){
            messageView->putln(
                fmt(_("Item type %1% of %2% cannot be restored. It's not a registered type."))
                % className % pluginName);
        } else {
            bool restored = false;

            item->setName(name);

            bool isRootItem = dynamic_pointer_cast<RootItem>(item);
            
            if(isRootItem){
                item = parentItem;
                restored = true;
            } else {
                messageView->putln(fmt(_("Restoring %1% \"%2%\"")) % className % name);
                messageView->flush();
                
                ValueNodePtr dataNode = archive.find("data");
                
                if(!dataNode->isValid()){
                    restored = true;
                    
                } else if(dataNode->type() == ValueNode::MAPPING){
                    Archive* dataArchive = static_cast<Archive*>(dataNode->toMapping());
                    dataArchive->inheritSharedInfoFrom(archive);
                    dataArchive->setCurrentParentItem(parentItem);
                    restored = item->restore(*dataArchive);
                }
                
                if(restored){
                    parentItem->addChildItem(item);
                }
            }
            
            if(!restored){
                messageView->putln(fmt(_("%1% \"%2%\" cannot be restored.")) % className % name);
                item = 0;
            }
        }
    }

    if(item){
        int id;
        if(archive.read("id", id) && (id >= 0)){
            archive.registerItemId(item.get(), id);
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
    
    return (item);
}
