/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_ITEM_TREE_ARCHIVER_H_INCLUDED
#define CNOID_GUIBASE_ITEM_TREE_ARCHIVER_H_INCLUDED

#include "Item.h"
#include "Archive.h"
#include "exportdecl.h"

namespace cnoid {

class MessageView;

class CNOID_EXPORT ItemTreeArchiver
{
public:
    ItemTreeArchiver();
    ~ItemTreeArchiver();
        
    ArchivePtr store(ArchivePtr parentArchive, ItemPtr topItem);
    bool restore(ArchivePtr archive, ItemPtr parentItem);

private:
    ArchivePtr storeIter(Archive& parentArchive, Item* item);
    bool restoreItemIter(Archive& archive, ItemPtr& parentItem);
    MessageView* messageView;
    int itemIdCounter;
};
}


#endif
