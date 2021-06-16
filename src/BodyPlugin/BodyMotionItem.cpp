/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "BodyMotionItem.h"
#include "BodyItem.h"
#include <cnoid/MultiSeqItemCreationPanel>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/ZMPSeq>
#include <cnoid/MessageView>
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
    Connection sigUpdateConnection;

    ExtraSeqItemInfo(const string& key, AbstractSeqItemPtr& item) : key(key), item(item) { }
    ~ExtraSeqItemInfo() {
        sigUpdateConnection.disconnect();
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

class BodyMotionItemImpl
{
public:
    BodyMotionItem* self;
        
    Connection jointPosSeqUpdateConnection;
    Connection linkPosSeqUpdateConnection;

    ExtraSeqItemInfoMap extraSeqItemInfoMap;
    vector<ExtraSeqItemInfoPtr> extraSeqItemInfos;
    Signal<void()> sigExtraSeqItemsChanged;
    Connection extraSeqsChangedConnection;

    BodyMotionItemImpl(BodyMotionItem* self);
    ~BodyMotionItemImpl();
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
        auto qseq = motionItem->jointPosSeq();
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

    initialized = true;
}


void BodyMotionItem::addExtraSeqItemFactory
(const std::string& key, std::function<AbstractSeqItem*(std::shared_ptr<AbstractSeq> seq)> factory)
{
    extraSeqItemFactories[key] = factory;
}


BodyMotionItem::BodyMotionItem()
    : bodyMotion_(new BodyMotion())
{
    impl = new BodyMotionItemImpl(this);
    impl->initialize();
}


BodyMotionItem::BodyMotionItem(std::shared_ptr<BodyMotion> bodyMotion)
    : bodyMotion_(bodyMotion)
{
    impl = new BodyMotionItemImpl(this);
    impl->initialize();
}


BodyMotionItem::BodyMotionItem(const BodyMotionItem& org)
    : AbstractSeqItem(org),
      bodyMotion_(new BodyMotion(*org.bodyMotion_))
{
    impl = new BodyMotionItemImpl(this);
    impl->initialize();
}


BodyMotionItemImpl::BodyMotionItemImpl(BodyMotionItem* self)
    : self(self)
{

}


void BodyMotionItemImpl::initialize()
{
    self->jointPosSeqItem_ = new MultiValueSeqItem(self->bodyMotion_->jointPosSeq());
    self->jointPosSeqItem_->setName("Joint");
    self->addSubItem(self->jointPosSeqItem_);

    jointPosSeqUpdateConnection =
        self->jointPosSeqItem_->sigUpdated().connect(
            [&](){ onSubItemUpdated(); });

    self->linkPosSeqItem_ = new MultiSE3SeqItem(self->bodyMotion_->linkPosSeq());
    self->linkPosSeqItem_->setName("Cartesian");
    self->addSubItem(self->linkPosSeqItem_);

    linkPosSeqUpdateConnection = 
        self->linkPosSeqItem_->sigUpdated().connect(
            [&](){ onSubItemUpdated(); });

    extraSeqsChangedConnection =
        self->bodyMotion_->sigExtraSeqsChanged().connect(
            [&](){ updateExtraSeqItems(); });

    updateExtraSeqItems();
}


BodyMotionItem::~BodyMotionItem()
{
    delete impl;
}


BodyMotionItemImpl::~BodyMotionItemImpl()
{
    extraSeqsChangedConnection.disconnect();
    jointPosSeqUpdateConnection.disconnect();
    linkPosSeqUpdateConnection.disconnect();
}


std::shared_ptr<AbstractSeq> BodyMotionItem::abstractSeq()
{
    return bodyMotion_;
}


void BodyMotionItem::notifyUpdate()
{
    impl->jointPosSeqUpdateConnection.block();
    jointPosSeqItem_->notifyUpdate();
    impl->jointPosSeqUpdateConnection.unblock();

    impl->linkPosSeqUpdateConnection.block();
    linkPosSeqItem_->notifyUpdate();
    impl->linkPosSeqUpdateConnection.unblock();

    vector<ExtraSeqItemInfoPtr>& extraSeqItemInfos = impl->extraSeqItemInfos;
    for(size_t i=0; i < extraSeqItemInfos.size(); ++i){
        ExtraSeqItemInfo* info = extraSeqItemInfos[i];
        info->sigUpdateConnection.block();
        info->item->notifyUpdate();
        info->sigUpdateConnection.unblock();
    }

    Item::notifyUpdate();
}


void BodyMotionItemImpl::onSubItemUpdated()
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


void BodyMotionItemImpl::updateExtraSeqItems()
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


Item* BodyMotionItem::doDuplicate() const
{
    return new BodyMotionItem(*this);
}


bool BodyMotionItem::store(Archive& archive)
{
    if(overwrite()){
        return archive.writeFileInformation(this);
    }
    return false;
}


bool BodyMotionItem::restore(const Archive& archive)
{
    return archive.loadFileTo(this);
}
