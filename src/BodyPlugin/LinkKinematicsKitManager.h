#ifndef CNOID_BODY_PLUGIN_LINK_KINEMATICS_KIT_MANAGER_H
#define CNOID_BODY_PLUGIN_LINK_KINEMATICS_KIT_MANAGER_H

#include <memory>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;
class LinkKinematicsKit;
class Link;
class InverseKinematics;
class SgNode;
class Mapping;

class CNOID_EXPORT LinkKinematicsKitManager
{
public:
    LinkKinematicsKitManager(BodyItem* bodyItem);
    ~LinkKinematicsKitManager();

    LinkKinematicsKit* getOrCreateKinematicsKit(Link* targetLink);
    LinkKinematicsKit* getOrCreateKinematicsKit(Link* targetLink, Link* baseLink);
    LinkKinematicsKit* getOrCreateKinematicsKit(Link* targetLink, std::shared_ptr<InverseKinematics> ik);

    SgNode* scene();

    bool storeState(Mapping& archive) const;
    bool restoreState(const Mapping& archive);

private:
    class Impl;
    Impl* impl;
};

}

#endif
