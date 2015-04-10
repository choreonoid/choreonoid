/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ITEM_TREE_ARCHIVER_H
#define CNOID_BASE_ITEM_TREE_ARCHIVER_H

#include "Archive.h"
#include "exportdecl.h"

namespace cnoid {

class Item;
class ItemTreeArchiverImpl;

class CNOID_EXPORT ItemTreeArchiver
{
public:
    ItemTreeArchiver();
    ~ItemTreeArchiver();
    void reset();
    ArchivePtr store(ArchivePtr parentArchive, Item* topItem);
    bool restore(ArchivePtr archive, Item* parentItem);
    int numArchivedItems() const;
    int numRestoredItems() const;

private:
    ItemTreeArchiverImpl* impl;
};

}

#endif
