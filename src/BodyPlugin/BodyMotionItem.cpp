#include "BodyMotionItem.h"
#include "BodyItem.h"
#include <cnoid/MultiSeqItemCreationPanel>
#include <cnoid/ItemManager>
#include <cnoid/MenuManager>
#include <cnoid/MultiSE3SeqItem>
#include <cnoid/MultiValueSeqItem>
#include <cnoid/ZMPSeq>
#include <cnoid/MessageView>
#include <cnoid/ItemTreeView>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

typedef std::function<AbstractSeqItem*(shared_ptr<AbstractSeq> seq)> ExtraSeqItemFactory;
typedef map<string, ExtraSeqItemFactory> ExtraSeqItemFactoryMap;
ExtraSeqItemFactoryMap extraSeqItemFactories;

struct ExtraSeqItemInfo : public Referenced
{
    string key;
    AbstractSeqItemPtr item;
    ScopedConnection sigUpdateConnection;

    ExtraSeqItemInfo(const string& key, AbstractSeqItemPtr& item) : key(key), item(item) { }
    ~ExtraSeqItemInfo() {
        item->removeFromParentItem();
    }
};

typedef ref_ptr<ExtraSeqItemInfo> ExtraSeqItemInfoPtr;
    
typedef std::map<std::string, ExtraSeqItemInfoPtr> ExtraSeqItemInfoMap;

class BodyMotionItemCreationPanel : public MultiSeqItemCreationPanel
{
public:
    BodyMotionItemCreationPanel();
    virtual void doExtraInitialization(AbstractSeqItem* protoItem, Item* parentItem) override;
    virtual void doExtraItemUpdate(AbstractSeqItem* protoItem, Item* parentItem) override;
};

}

namespace cnoid {

class BodyMotionItem::Impl
{
public:
    BodyMotionItem* self;
    ExtraSeqItemInfoMap extraSeqItemInfoMap;
    vector<ExtraSeqItemInfoPtr> extraSeqItemInfos;
    Signal<void()> sigExtraSeqItemsChanged;
    ScopedConnection extraSeqsChangedConnection;

    Impl(BodyMotionItem* self);
    void initialize();
    void onSubItemUpdated();
    void updateExtraSeqItems();
};

}


BodyMotionItemCreationPanel::BodyMotionItemCreationPanel()
    : MultiSeqItemCreationPanel(_("Number of joints"))
{

}


void BodyMotionItemCreationPanel::doExtraInitialization(AbstractSeqItem* protoItem, Item* parentItem)
{
    BodyItemPtr bodyItem = dynamic_cast<BodyItem*>(parentItem);
    if(!bodyItem){
        bodyItem = parentItem->findOwnerItem<BodyItem>();
    }
    if(bodyItem){
        auto motionItem = static_cast<BodyMotionItem*>(protoItem);
        auto jointPosSeq = motionItem->motion()->jointPosSeq();
        int numJoints = bodyItem->body()->numJoints();
        if(numJoints != jointPosSeq->numParts()){
            jointPosSeq->setNumParts(numJoints, true);
        }
    }
}


void BodyMotionItemCreationPanel::doExtraItemUpdate(AbstractSeqItem* protoItem, Item* parentItem)
{
    BodyItemPtr bodyItem = dynamic_cast<BodyItem*>(parentItem);
    if(!bodyItem){
        bodyItem = parentItem->findOwnerItem<BodyItem>();
    }
    if(bodyItem){
        auto motionItem = static_cast<BodyMotionItem*>(protoItem);
        auto body = bodyItem->body();
        auto qseq = motionItem->motion()->jointPosSeq();
        int n = std::min(body->numJoints(), qseq->numParts());
        for(int i=0; i < n; ++i){
            auto joint = body->joint(i);
            if(joint->q_initial() != 0.0){
                auto part = qseq->part(i);
                std::fill(part.begin(), part.end(), joint->q_initial());
            }
        }
    }
}


void BodyMotionItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    
    if(initialized){
        return;
    }
    
    ItemManager& im = ext->itemManager();
    
    im.registerClass<BodyMotionItem, AbstractSeqItem>(N_("BodyMotionItem"));

    im.addCreationPanel<BodyMotionItem>(new BodyMotionItemCreationPanel);

    im.addLoaderAndSaver<BodyMotionItem>(
        _("Body Motion"), "BODY-MOTION-YAML", "seq;yaml",
        [](BodyMotionItem* item, const std::string& filename, std::ostream& os, Item* /* parentItem */){
            return item->motion()->load(filename, os);
        },
        [](BodyMotionItem* item, const std::string& filename, std::ostream& os, Item* /* parentItem */){
            return item->motion()->save(filename, os);
        });

    im.addSaver<BodyMotionItem>(
        _("Body Motion (version 1.0)"), "BODY-MOTION-YAML", "seq;yaml",
        [](BodyMotionItem* item, const std::string& filename, std::ostream& os, Item* /* parentItem */){
            return item->motion()->save(filename, 1.0, os);
        });

    addExtraSeqItemFactory(
        BodyMotion::linkPosSeqKey(),
        [](std::shared_ptr<AbstractSeq> seq) -> AbstractSeqItem* {
            MultiSE3SeqItem* item = nullptr;
            if(auto linkPosSeq = dynamic_pointer_cast<MultiSE3Seq>(seq)){
                item = new MultiSE3SeqItem(linkPosSeq);
                item->setName("Cartesian");
            }
            return item;
        });
    
    addExtraSeqItemFactory(
        BodyMotion::jointPosSeqKey(),
        [](std::shared_ptr<AbstractSeq> seq) -> AbstractSeqItem* {
            MultiValueSeqItem* item = nullptr;
            if(auto jointPosSeq = dynamic_pointer_cast<MultiValueSeq>(seq)){
                item = new MultiValueSeqItem(jointPosSeq);
                item->setName("Joint");
            }
            return item;
        });

    ItemTreeView::customizeContextMenu<BodyMotionItem>(
        [](BodyMotionItem* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction){
            menuManager.setPath("/").setPath(_("Data conversion"));
            menuManager.addItem(_("Generate old-format position data items"))->sigTriggered().connect(
                [item](){ item->motion()->updateLinkPosSeqAndJointPosSeqWithBodyPositionSeq(); });
            menuManager.addItem(_("Restore position data from old-format data items"))->sigTriggered().connect(
                [item](){ item->motion()->updateBodyPositionSeqWithLinkPosSeqAndJointPosSeq(); });
            menuManager.setPath("/");
            menuManager.addSeparator();
            menuFunction.dispatchAs<Item>(item);
        });

    initialized = true;
}


void BodyMotionItem::addExtraSeqItemFactory
(const std::string& key, std::function<AbstractSeqItem*(std::shared_ptr<AbstractSeq> seq)> factory)
{
    extraSeqItemFactories[key] = factory;
}


BodyMotionItem::BodyMotionItem()
    : bodyMotion_(new BodyMotion),
      isBodyJointVelocityUpdateEnabled_(false)
{
    setAttribute(Reloadable);
    impl = new Impl(this);
    impl->initialize();
}


BodyMotionItem::BodyMotionItem(std::shared_ptr<BodyMotion> bodyMotion)
    : bodyMotion_(bodyMotion),
      isBodyJointVelocityUpdateEnabled_(false)
{
    impl = new Impl(this);
    impl->initialize();
}


BodyMotionItem::BodyMotionItem(const BodyMotionItem& org)
    : AbstractSeqItem(org),
      bodyMotion_(new BodyMotion(*org.bodyMotion_)),
      isBodyJointVelocityUpdateEnabled_(org.isBodyJointVelocityUpdateEnabled_)
{
    impl = new Impl(this);
    impl->initialize();
}


BodyMotionItem::Impl::Impl(BodyMotionItem* self)
    : self(self)
{

}


void BodyMotionItem::Impl::initialize()
{
    extraSeqsChangedConnection =
        self->bodyMotion_->sigExtraSeqsChanged().connect(
            [&](){ updateExtraSeqItems(); });

    updateExtraSeqItems();
}


BodyMotionItem::~BodyMotionItem()
{
    delete impl;
}


Item* BodyMotionItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new BodyMotionItem(*this);
}


std::shared_ptr<AbstractSeq> BodyMotionItem::abstractSeq()
{
    return bodyMotion_;
}


void BodyMotionItem::notifyUpdate()
{
    vector<ExtraSeqItemInfoPtr>& extraSeqItemInfos = impl->extraSeqItemInfos;
    for(size_t i=0; i < extraSeqItemInfos.size(); ++i){
        ExtraSeqItemInfo* info = extraSeqItemInfos[i];
        info->sigUpdateConnection.block();
        info->item->notifyUpdate();
        info->sigUpdateConnection.unblock();
    }

    Item::notifyUpdate();
}


void BodyMotionItem::Impl::onSubItemUpdated()
{
    self->suggestFileUpdate();
    self->Item::notifyUpdate();
}


int BodyMotionItem::numExtraSeqItems() const
{
    return impl->extraSeqItemInfos.size();
}


const std::string& BodyMotionItem::extraSeqKey(int index) const
{
    return impl->extraSeqItemInfos[index]->key;
}


AbstractSeqItem* BodyMotionItem::extraSeqItem(int index)
{
    return impl->extraSeqItemInfos[index]->item;
}


const AbstractSeqItem* BodyMotionItem::extraSeqItem(int index) const
{
    return impl->extraSeqItemInfos[index]->item;
}


SignalProxy<void()> BodyMotionItem::sigExtraSeqItemsChanged()
{
    return impl->sigExtraSeqItemsChanged;
}


void BodyMotionItem::updateExtraSeqItems()
{
    impl->updateExtraSeqItems();
}


void BodyMotionItem::Impl::updateExtraSeqItems()
{
    extraSeqItemInfos.clear();

    BodyMotion& bodyMotion = *self->bodyMotion_;
    BodyMotion::ConstSeqIterator p;
    for(p = bodyMotion.extraSeqBegin(); p != bodyMotion.extraSeqEnd(); ++p){
        const string& key = p->first;
        auto newSeq = p->second;
        AbstractSeqItemPtr newItem;
        ExtraSeqItemInfoMap::iterator p = extraSeqItemInfoMap.find(key);
        if(p != extraSeqItemInfoMap.end()){
            ExtraSeqItemInfo* info = p->second;
            AbstractSeqItemPtr& prevItem = info->item;
            if(typeid(prevItem->abstractSeq().get()) == typeid(newSeq.get())){
                extraSeqItemInfos.push_back(info);
                newItem = prevItem;
            }
        }
        if(!newItem){
            ExtraSeqItemFactoryMap::iterator q = extraSeqItemFactories.find(key);
            if(q != extraSeqItemFactories.end()){
                ExtraSeqItemFactory& factory = q->second;
                newItem = factory(newSeq);
                if(newItem){
                    self->addSubItem(newItem);
                    ExtraSeqItemInfo* info = new ExtraSeqItemInfo(key, newItem);
                    info->sigUpdateConnection =
                        newItem->sigUpdated().connect([&](){ onSubItemUpdated(); });
                    extraSeqItemInfos.push_back(info);
                }
            }
        }
    }
    extraSeqItemInfoMap.clear();
    for(size_t i=0; i < extraSeqItemInfos.size(); ++i){
        ExtraSeqItemInfo* info = extraSeqItemInfos[i];
        extraSeqItemInfoMap.insert(make_pair(info->key, info));
    }

    sigExtraSeqItemsChanged();
}


bool BodyMotionItem::onChildItemAboutToBeAdded(Item* childItem_, bool isManualOperation)
{
    if(isManualOperation){
        if(auto seqItem = dynamic_cast<AbstractSeqItem*>(childItem_)){
            if(!dynamic_cast<BodyMotionItem*>(seqItem)){
                bool existingFound = false;
                for(Item* item = childItem(); item; item = item->nextItem()){
                    if(item->isSubItem() && item->name() == seqItem->name()){
                        if(auto orgSeqItem = dynamic_cast<AbstractSeqItem*>(item)){
                            existingFound = true;
                            if(showConfirmDialog(
                                   _("Confirm"),
                                   format(_("Do you overwrite the data of \"{}\"?"),
                                          orgSeqItem->displayName()))){
                                *orgSeqItem->abstractSeq() = *seqItem->abstractSeq();
                                return false; // Not replace the item itself
                            }
                        }
                    }
                }
                if(!existingFound){
                    if(showConfirmDialog(
                           _("Confirm"),
                           format(_("Do you add the data of \"{0}\" to \"{1}\" as a sequence data element?"),
                                  childItem_->displayName(), this->displayName()))){
                        motion()->setExtraSeq(seqItem->name(), seqItem->abstractSeq());
                        return false; // Not replace the item itself
                    }
                }
            }
        }
    }
    return true;
}


void BodyMotionItem::doPutProperties(PutPropertyFunction& putProperty)
{
    AbstractSeqItem::doPutProperties(putProperty);

    auto pseq = bodyMotion_->positionSeq();
    
    putProperty(_("Number of link positions"), pseq->numLinkPositionsHint());
    putProperty(_("Number of joint displacements"), pseq->numJointDisplacementsHint());

    putProperty(_("Body joint velocity update"), isBodyJointVelocityUpdateEnabled_,
                changeProperty(isBodyJointVelocityUpdateEnabled_));
}


bool BodyMotionItem::store(Archive& archive)
{
    if(AbstractSeqItem::store(archive)){
        if(isBodyJointVelocityUpdateEnabled_){
            archive.write("is_body_joint_velocity_update_enabled", true);
        }
        return true;
    }
    return false;
}


bool BodyMotionItem::restore(const Archive& archive)
{
    isBodyJointVelocityUpdateEnabled_ = archive.get("is_body_joint_velocity_update_enabled", false);
    return AbstractSeqItem::restore(archive);
}
