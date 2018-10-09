/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ZMPSeqItem.h"
#include "BodyItem.h"
#include "BodyMotionItem.h"
#include "BodyMotionEngine.h"
#include <cnoid/ItemManager>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using boost::format;

namespace {

AbstractSeqItem* createZMPSeqItem(AbstractSeqPtr seq)
{
    ZMPSeqPtr zmpseq = dynamic_pointer_cast<ZMPSeq>(seq);
    if(zmpseq){
        auto item = new ZMPSeqItem(zmpseq);
        item->setName("ZMP");
        return item;
    }
    return nullptr;
}


class ZMPSeqEngine : public TimeSyncItemEngine
{
    ZMPSeqPtr seq;
    BodyItemPtr bodyItem;
public:
        
    ZMPSeqEngine(ZMPSeqItem* seqItem, BodyItem* bodyItem)
        : seq(seqItem->zmpseq()), bodyItem(bodyItem)
        {
            seqItem->sigUpdated().connect(std::bind(&TimeSyncItemEngine::notifyUpdate, this));
        }

    virtual bool onTimeChanged(double time)
        {
            bool isValidTime = false;
            if(!seq->empty()){
                const Vector3& zmp = seq->at(seq->clampFrameIndex(seq->frameOfTime(time), isValidTime));
                if(seq->isRootRelative()){
                    bodyItem->setZmp(bodyItem->body()->rootLink()->T() * zmp);
                } else {
                    bodyItem->setZmp(zmp);
                }
            }
            return isValidTime;
        }
};


TimeSyncItemEngine* createZMPSeqEngine(BodyItem* bodyItem, AbstractSeqItem* seqItem)
{
    ZMPSeqItem* item = dynamic_cast<ZMPSeqItem*>(seqItem);
    if(item){
        return new ZMPSeqEngine(item, bodyItem);
    }
    return 0;
}

}


void ZMPSeqItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<ZMPSeqItem>(N_("ZMPSeqItem"));
    
    BodyMotionItem::addExtraSeqItemFactory(ZMPSeq::key(), createZMPSeqItem);
    BodyMotionEngine::addExtraSeqEngineFactory(ZMPSeq::key(), createZMPSeqEngine);
}


ZMPSeqItem::ZMPSeqItem()
    : Vector3SeqItem(std::make_shared<ZMPSeq>())
{
    zmpseq_ = static_pointer_cast<ZMPSeq>(seq());
}


ZMPSeqItem::ZMPSeqItem(ZMPSeqPtr seq)
    : Vector3SeqItem(seq),
      zmpseq_(seq)
{

}


ZMPSeqItem::ZMPSeqItem(const ZMPSeqItem& org)
    : Vector3SeqItem(org, std::make_shared<ZMPSeq>(*org.zmpseq_))
{
    zmpseq_ = static_pointer_cast<ZMPSeq>(seq());
}


ZMPSeqItem::~ZMPSeqItem()
{

}


bool ZMPSeqItem::makeRootRelative(bool on)
{
    BodyMotionItem* bodyMotionItem = dynamic_cast<BodyMotionItem*>(parentItem());
    if(bodyMotionItem){
        if(cnoid::makeRootRelative(*zmpseq_, *bodyMotionItem->motion(), on)){
            mvout() << (format(_("%1% of %2% has been converted to %3%."))
                        % name() % bodyMotionItem->name()
                        % (on ? _("the root relative coordinate") : _("the global coordinate")))
                    << endl;
            return true;
        }
    }
    mvout() << (format(_("%1%'s coordinate system cannot be changed "
                         "because there is no root link motion associated with %1%."))
                % name()) << endl;
    return false;
}


Item* ZMPSeqItem::doDuplicate() const
{
    return new ZMPSeqItem(*this);
}


void ZMPSeqItem::doPutProperties(PutPropertyFunction& putProperty)
{
    AbstractSeqItem::doPutProperties(putProperty);
    putProperty(_("Root relative"), zmpseq_->isRootRelative(),
                std::bind(&ZMPSeqItem::makeRootRelative, this, _1));
}
