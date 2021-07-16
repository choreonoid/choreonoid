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

    /**
       A string for identifying the folder category.
       This value is mainly used in the system, and a user does not specify this manually.
    */
    const std::string& category() const { return category_; }
    void setCategory(const std::string& category){ category_ = category; }

protected:
    virtual Item* doDuplicate() const override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    std::string category_;
};

typedef ref_ptr<FolderItem> FolderItemPtr;
}

#endif
