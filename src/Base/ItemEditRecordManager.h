#ifndef CNOID_BASE_ITEM_EDIT_RECORD_MANAGER_H
#define CNOID_BASE_ITEM_EDIT_RECORD_MANAGER_H

#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;

class CNOID_EXPORT ItemEditRecordManager
{
public:
    static void initializeClass(ExtensionManager* ext);

    ItemEditRecordManager(const ItemEditRecordManager&) = delete;
    ItemEditRecordManager(ItemEditRecordManager&&) = delete;
    ItemEditRecordManager& operator=(const ItemEditRecordManager&) = delete;
    ItemEditRecordManager& operator=(ItemEditRecordManager&&) = delete;

    ~ItemEditRecordManager();

    class Impl;

private:
    ItemEditRecordManager();
    
    Impl* impl;
};

}

#endif
