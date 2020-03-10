#include "LinkShapeItem.h"
#include "BodyItem.h"
#include <cnoid/ItemManager>
#include <cnoid/SceneItemFileIO>
#include <cnoid/SceneGraph>
#include <cnoid/Archive>
#include "gettext.h"

using namespace std;
using namespace cnoid;


namespace cnoid {

class LinkShapeItem::Impl
{
public:
    SgPosTransformPtr topNode;
    Signal<void()> sigOffsetChanged;
    BodyItem* bodyItem;
    
    Impl();
    Impl(const Impl& org);
    void setShape(SgNode* shape);
    void setBodyItem(BodyItem* newBodyItem);
};

class LinkShapeItem::FileIO : public SceneItemFileIO
{
public:
    FileIO()
    {
        setApi(Load | Options);
        setCaption(_("Link Shape"));
        setInterfaceLevel(Internal);
    }

    virtual Item* createItem() override
    {
        return new LinkShapeItem;
    }
    
    virtual bool load(Item* item, const std::string& filename) override
    {
        if(auto shape = loadScene(filename)){
            static_cast<LinkShapeItem*>(item)->impl->setShape(shape);
            return true;
        }
        return false;
    }
};

}


void LinkShapeItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<LinkShapeItem>(N_("LinkShapeItem"));
    im.registerFileIO<LinkShapeItem>(new LinkShapeItem::FileIO);
}


LinkShapeItem::LinkShapeItem()
{
    impl = new Impl;
}


LinkShapeItem::Impl::Impl()
{
    topNode = new SgPosTransform;
    bodyItem = nullptr;
}


LinkShapeItem::LinkShapeItem(const LinkShapeItem& org)
    : Item(org)
{
    impl = new Impl(*org.impl);
}


LinkShapeItem::Impl::Impl(const Impl& org)
    : Impl()
{

}


LinkShapeItem::~LinkShapeItem()
{
    delete impl;
}



Item* LinkShapeItem::doDuplicate() const
{
    return new LinkShapeItem(*this);
}


void LinkShapeItem::setShape(SgNode* shape)
{
    impl->setShape(shape);
}


void LinkShapeItem::Impl::setShape(SgNode* shape)
{
    topNode->clearChildren();
    topNode->addChild(shape);
}


void LinkShapeItem::onPositionChanged()
{
    impl->setBodyItem(findOwnerItem<BodyItem>());
}


void LinkShapeItem::Impl::setBodyItem(BodyItem* newBodyItem)
{
    if(newBodyItem != bodyItem){
        if(bodyItem){
            bodyItem->body()->rootLink()->removeShapeNode(topNode, true);
        }
        if(newBodyItem){
            auto rootLink = newBodyItem->body()->rootLink();
            rootLink->addShapeNode(topNode, true);
        }
        bodyItem = newBodyItem;
    }
}


Position LinkShapeItem::shapeOffset() const
{
    return impl->topNode->T();
}


void LinkShapeItem::setShapeOffset(const Position& T)
{
    impl->topNode->setPosition(T);
}


Position LinkShapeItem::getLocation() const
{
    return impl->topNode->T();
}


bool LinkShapeItem::prefersLocalLocation() const
{
    return true;
}


SignalProxy<void()> LinkShapeItem::sigLocationChanged()
{
    return impl->sigOffsetChanged;
}


void LinkShapeItem::setLocationEditable(bool on)
{

}


void LinkShapeItem::setLocation(const Position& T)
{

}


LocatableItem* LinkShapeItem::getParentLocatableItem()
{
    return impl->bodyItem;
}


void LinkShapeItem::doPutProperties(PutPropertyFunction& putProperty)
{
    Item::doPutProperties(putProperty);
}


bool LinkShapeItem::store(Archive& archive)
{
    return archive.writeFileInformation(this);
}


bool LinkShapeItem::restore(const Archive& archive)
{
    return archive.loadFileTo(this);
}
