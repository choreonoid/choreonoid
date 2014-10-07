/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ITEM_TREE_ARCHIVER_H_INCLUDED
#define CNOID_BASE_ITEM_TREE_ARCHIVER_H_INCLUDED

#include "Archive.h"
#include "exportdecl.h"

namespace cnoid {

class Item;
class MessageView;

class CNOID_EXPORT ItemTreeArchiver
{
public:
    ItemTreeArchiver();
    ~ItemTreeArchiver();
        
    ArchivePtr store(ArchivePtr parentArchive, Item* topItem);
    bool restore(ArchivePtr archive, Item* parentItem);

private:
    ArchivePtr storeIter(Archive& parentArchive, Item* item);
    bool restoreItemIter(Archive& archive, Item* parentItem);
    MessageView* messageView;
    int itemIdCounter;
};
}


#endif
