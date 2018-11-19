/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_FOLDER_ITEM_H
#define CNOID_BASE_FOLDER_ITEM_H

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
    virtual Item* doDuplicate() const override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
};

typedef ref_ptr<FolderItem> FolderItemPtr;
}

#endif
