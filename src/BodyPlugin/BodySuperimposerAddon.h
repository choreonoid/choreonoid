#ifndef CNOID_BODY_PLUGIN_BODY_SUPERIMPOSER_ADDON_H
#define CNOID_BODY_PLUGIN_BODY_SUPERIMPOSER_ADDON_H

#include <cnoid/ItemAddon>
#include "exportdecl.h"

namespace cnoid {

class Body;

class CNOID_EXPORT BodySuperimposerAddon : public ItemAddon
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    BodySuperimposerAddon();
    virtual ~BodySuperimposerAddon();

    virtual bool setOwnerItem(Item* item) override;

    int numSuperimposedBodies() const;
    Body* superimposedBody(int index);
    void setTransparency(float transparency);
    void updateSuperimposition();
    bool updateSuperimposition(
        std::function<bool()> setReferenceConfigurationToOrgBodiesTransiently);
    void clearSuperimposition();
    
protected:
    BodySuperimposerAddon(const BodySuperimposerAddon& org);
    virtual ItemAddon* doClone(Item* newItem, CloneMap* cloneMap) const override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
            
private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<BodySuperimposerAddon> BodySuperimposerAddonPtr;

}

#endif
