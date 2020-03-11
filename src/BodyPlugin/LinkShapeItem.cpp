#include "LinkShapeItem.h"
#include "BodyItem.h"
#include <cnoid/ItemManager>
#include <cnoid/SceneItemFileIO>
#include <cnoid/SceneGraph>
#include <cnoid/Archive>
#include <cnoid/EigenArchive>
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
    setLocationEditable(false);
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
    setLocationEditable(false);
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
    impl->topNode->notifyUpdate();
    impl->sigOffsetChanged();
    notifyUpdate();
}


Position LinkShapeItem::getLocation() const
{
    if(impl->bodyItem){
        return impl->bodyItem->body()->rootLink()->T() * shapeOffset();
    } else {
        return shapeOffset();
    }
}


bool LinkShapeItem::prefersLocalLocation() const
{
    return true;
}


SignalProxy<void()> LinkShapeItem::sigLocationChanged()
{
    return impl->sigOffsetChanged;
}


void LinkShapeItem::setLocation(const Position& T)
{
    if(impl->bodyItem){
        impl->topNode->setPosition(
            impl->bodyItem->body()->rootLink()->T().inverse(Eigen::Isometry) * T);
    } else {
        impl->topNode->setPosition(T);
    }
    impl->topNode->notifyUpdate();
    notifyUpdate();
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
    if(archive.writeFileInformation(this)){
        auto& T = impl->topNode->T();
        if(T.translation() != Vector3::Zero()){
            write(archive, "translation", T.translation());
        }
        Matrix3 R = T.linear();
        if(!R.isApprox(Matrix3::Identity())){
            writeDegreeAngleAxis(archive, "rotation", AngleAxis(R));
        }
        return true;
    }
    return false;
}


bool LinkShapeItem::restore(const Archive& archive)
{
    if(archive.loadFileTo(this)){
        Vector3 translation;
        if(read(archive, "translation", translation)){
            impl->topNode->setTranslation(translation);
        }
        AngleAxis rot;
        if(readDegreeAngleAxis(archive, "rotation", rot)){
            impl->topNode->setRotation(rot);
        }
        return true;
    }
    return false;
}
