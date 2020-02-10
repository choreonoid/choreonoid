#ifndef CNOID_BODY_PLUGIN_BODY_SUPERIMPOSER_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_SUPERIMPOSER_ITEM_H

#include <cnoid/Item>
#include <cnoid/RenderableItem>
#include "exportdecl.h"

namespace cnoid {

class Body;

class CNOID_EXPORT BodySuperimposerItem : public Item, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    BodySuperimposerItem();
    BodySuperimposerItem(const BodySuperimposerItem& org);
    virtual ~BodySuperimposerItem();

    virtual void setName(const std::string& name) override;

    // RenderableItem's function
    virtual SgNode* getScene() override;

    int numSuperimposedBodies() const;
    Body* superimposedBody(int index);

    void setTransparency(float transparency);

    void updateSuperimposition();
    void clearSuperimposition();

protected:
    virtual Item* doDuplicate() const override;
    virtual void onPositionChanged() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
            
private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<BodySuperimposerItem> BodySuperimposerItemPtr;

}

#endif
