#ifndef CNOID_MOCAP_PLUGIN_MOCAP_MAPPING_ITEM_H
#define CNOID_MOCAP_PLUGIN_MOCAP_MAPPING_ITEM_H

#include "MocapMapping.h"
#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MocapMappingItem : public Item
{
public:
    static void initialize(ExtensionManager* ext);

    MocapMappingItem();
    MocapMappingItem(const MocapMappingItem& org);

    MocapMappingPtr mocapMapping() { return mocapMapping_; }

    bool loadMocapMapping(const std::string& filename, std::ostream& os);

protected:
    virtual Item* doDuplicate() const;
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
    virtual void doPutProperties(PutPropertyFunction& putProperty);

private:
    MocapMappingPtr mocapMapping_;

    virtual ~MocapMappingItem();
};

typedef ref_ptr<MocapMappingItem> MocapMappingItemPtr;

}

#endif
