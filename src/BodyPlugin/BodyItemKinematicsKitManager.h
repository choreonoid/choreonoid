#ifndef CNOID_BODY_PLUGIN_BODY_ITEM_KINEMATICS_KIT_MANAGER_H
#define CNOID_BODY_PLUGIN_BODY_ITEM_KINEMATICS_KIT_MANAGER_H

#include <memory>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;
class BodyItemKinematicsKit;
class Link;
class Mapping;

class CNOID_EXPORT BodyItemKinematicsKitManager
{
public:
    BodyItemKinematicsKitManager(BodyItem* bodyItem);
    ~BodyItemKinematicsKitManager();

    BodyItemKinematicsKit* getCurrentKinematicsKit(Link* targetLink);
    BodyItemKinematicsKit* findPresetKinematicsKit(Link* targetLink = nullptr);
    
    bool storeState(Mapping& archive) const;
    bool restoreState(const Mapping& archive);

private:
    class Impl;
    Impl* impl;
};

}

#endif
