#ifndef CNOID_BASE_SUB_PROJECT_ITEM_H
#define CNOID_BASE_SUB_PROJECT_ITEM_H

#include "Item.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SubProjectItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    SubProjectItem();
    virtual ~SubProjectItem();

    bool isSavingSubProject() const;

    enum SaveMode { MANUAL_SAVE, AUTOMATIC_SAVE, N_SAVE_MODE };
    void setSaveMode(int mode);
    int saveMode() const;

protected:
    SubProjectItem(const SubProjectItem& org);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual void onConnectedToRoot() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<SubProjectItem> SubProjectItemPtr;
}

#endif
