/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ZMPSeqItem.h"
#include "BodyItem.h"
#include "BodyMotionItem.h"
#include "BodyMotionEngine.h"
#include <cnoid/ItemManager>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

AbstractSeqItemPtr createZMPSeqItem(AbstractSeqPtr seq)
{
    ZMPSeqPtr zmpseq = dynamic_pointer_cast<ZMPSeq>(seq);
    return zmpseq ? new ZMPSeqItem(zmpseq) : 0;
}


class ZMPSeqEngine : public TimeSyncItemEngine
{
    ZMPSeqPtr seq;
    BodyItemPtr bodyItem;
public:
        
    ZMPSeqEngine(ZMPSeqItemPtr seqItem, BodyItemPtr bodyItem)
        : seq(seqItem->zmpseq()), bodyItem(bodyItem)
        {
            seqItem->sigUpdated().connect(bind(&TimeSyncItemEngine::notifyUpdate, this));
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


TimeSyncItemEnginePtr createZMPSeqEngine(BodyItemPtr bodyItem, AbstractSeqItemPtr seqItem)
{
    ZMPSeqItemPtr item = dynamic_pointer_cast<ZMPSeqItem>(seqItem);
    if(item){
        return make_shared<ZMPSeqEngine>(item, bodyItem);
    }
    return TimeSyncItemEnginePtr();
}
}


void ZMPSeqItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<ZMPSeqItem>(N_("ZMPSeqItem"));
    
    BodyMotionItem::addExtraSeqItemFactory(ZMPSeq::key(), createZMPSeqItem);
    BodyMotionEngine::addExtraSeqEngineFactory(ZMPSeq::key(), createZMPSeqEngine);
}


ZMPSeqItem::ZMPSeqItem()
    : Vector3SeqItem(make_shared<ZMPSeq>())
{
    zmpseq_ = static_pointer_cast<ZMPSeq>(seq());
}


ZMPSeqItem::ZMPSeqItem(ZMPSeqPtr seq)
    : Vector3SeqItem(seq),
      zmpseq_(seq)
{

}


ZMPSeqItem::ZMPSeqItem(const ZMPSeqItem& org)
    : Vector3SeqItem(org, make_shared<ZMPSeq>(*org.zmpseq_))
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


ItemPtr ZMPSeqItem::doDuplicate() const
{
    return new ZMPSeqItem(*this);
}


void ZMPSeqItem::doPutProperties(PutPropertyFunction& putProperty)
{
    AbstractSeqItem::doPutProperties(putProperty);
    putProperty(_("Root relative"), zmpseq_->isRootRelative(),
                bind(&ZMPSeqItem::makeRootRelative, this, _1));
}
