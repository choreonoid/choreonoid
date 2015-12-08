/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_FOLDER_ITEM_H_INCLUDED
#define CNOID_GUIBASE_FOLDER_ITEM_H_INCLUDED

#include "Item.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT FolderItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    FolderItem();
    FolderItem(const FolderItem& org);
    virtual ~FolderItem();

protected:
    virtual Item* doDuplicate() const;
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
};

typedef ref_ptr<FolderItem> FolderItemPtr;
}

#endif
