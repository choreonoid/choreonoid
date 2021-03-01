#ifndef CNOID_BODY_PLUGIN_BODY_EDIT_RECORD_MANAGER_H
#define CNOID_BODY_PLUGIN_BODY_EDIT_RECORD_MANAGER_H

#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;

class CNOID_EXPORT BodyEditRecordManager
{
public:
    static void initializeClass(ExtensionManager* ext);

    BodyEditRecordManager(const BodyEditRecordManager&) = delete;
    BodyEditRecordManager(BodyEditRecordManager&&) = delete;
    BodyEditRecordManager& operator=(const BodyEditRecordManager&) = delete;
    BodyEditRecordManager& operator=(BodyEditRecordManager&&) = delete;

    ~BodyEditRecordManager();

    class Impl;

private:
    BodyEditRecordManager();
    
    Impl* impl;
};

}

#endif
