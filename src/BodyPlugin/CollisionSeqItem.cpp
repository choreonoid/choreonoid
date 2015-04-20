/**
   @file
   @author Shizuko Hattori
*/

#include "CollisionSeqItem.h"
#include <cnoid/ItemManager>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

}


namespace cnoid {

class CollisionSeqItemImpl
{
public:
    CollisionSeqItem* self;

    CollisionSeqItemImpl(CollisionSeqItem* self);
    ~CollisionSeqItemImpl();
    void initialize();

};
}


void CollisionSeqItem::initislizeClass(ExtensionManager* ext)
{
    static bool initialized = false;

    if(initialized){
        return;
    }

    ItemManager& im = ext->itemManager();

    im.registerClass<CollisionSeqItem>(N_("CollisionSeqItem"));
/*
    im.addCreationPanel<BodyMotionItem>(new MultiSeqItemCreationPanel(_("Number of joints")));
    im.addCreationPanelPreFilter<BodyMotionItem>(bodyMotionItemPreFilter);
    //im.addCreationPanelPostFilter<BodyMotionItem>(bodyMotionItemPostFilter);

    im.addLoaderAndSaver<BodyMotionItem>(
        _("Body Motion"), "BODY-MOTION-YAML", "yaml",
        boost::bind(loadStandardYamlFormat, _1, _2, _3),  boost::bind(saveAsStandardYamlFormat, _1, _2, _3));
*/
    initialized = true;
}


CollisionSeqItem::CollisionSeqItem()
    : collisionSeq_(new CollisionSeq())
{
    impl = new CollisionSeqItemImpl(this);
}


CollisionSeqItem::CollisionSeqItem(const CollisionSeqItem& org)
    : AbstractMultiSeqItem(org),
      collisionSeq_(new CollisionSeq(*org.collisionSeq_))
{
    impl = new CollisionSeqItemImpl(this);
}


CollisionSeqItemImpl::CollisionSeqItemImpl(CollisionSeqItem* self)
    :self(self)
{
    initialize();
}


void CollisionSeqItemImpl::initialize()
{

}


CollisionSeqItem::~CollisionSeqItem()
{
    delete impl;
}


CollisionSeqItemImpl::~CollisionSeqItemImpl()
{

}


AbstractMultiSeqPtr CollisionSeqItem::abstractMultiSeq()
{
    return collisionSeq_;
}


ItemPtr CollisionSeqItem::doDuplicate() const
{
    return new CollisionSeqItem(*this);
}


bool CollisionSeqItem::store(Archive& archive)
{
    return true;
}


bool CollisionSeqItem::restore(const Archive& archive)
{
    return true;
}


