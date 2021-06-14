/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "AbstractSeqItem.h"
#include "ItemManager.h"
#include "Archive.h"
#include "PutPropertyFunction.h"
#include "gettext.h"

using namespace cnoid;


void AbstractSeqItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerAbstractClass<AbstractSeqItem>();
}


AbstractSeqItem::AbstractSeqItem()
{

}


AbstractSeqItem::AbstractSeqItem(const AbstractSeqItem& org)
    : Item(org)
{

}


AbstractSeqItem::~AbstractSeqItem()
{

}


void AbstractSeqItem::doPutProperties(PutPropertyFunction& putProperty)
{
    auto seq = abstractSeq();

    putProperty(_("Frame rate"), seq->getFrameRate());

    putProperty(_("Offset time"), seq->getOffsetTime(),
                [&](double value){
                    abstractSeq()->setOffsetTime(value);
                    return true;
                });

    putProperty(_("Number of frames"), seq->getNumFrames(),
                [&](int value){
                    if(value >= 0){
                        abstractSeq()->setNumFrames(value);
                        suggestFileUpdate();
                        return true;
                    }
                    return false;
                });
    
    putProperty(_("Time length"), seq->getTimeLength(),
                [&](double value){
                    if(value >= 0){
                        abstractSeq()->setTimeLength(value);
                        suggestFileUpdate();
                        return true;
                    }
                    return false;
                });
                    
    putProperty.decimals(3)(_("Time step"), seq->getTimeStep());
}


bool AbstractSeqItem::store(Archive& archive)
{
    if(overwrite()){
        archive.writeFileInformation(this);
    }
    archive.write("offsetTime", abstractSeq()->getOffsetTime());
    return true;
}


bool AbstractSeqItem::restore(const Archive& archive)
{
    double offsetTime;
    if(archive.read("offsetTime", offsetTime)){
        abstractSeq()->setOffsetTime(offsetTime);
    }
    return archive.loadFileTo(this);
}


void AbstractMultiSeqItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerAbstractClass<AbstractMultiSeqItem, AbstractSeqItem>();
}


AbstractMultiSeqItem::AbstractMultiSeqItem()
{

}


AbstractMultiSeqItem::AbstractMultiSeqItem(const AbstractMultiSeqItem& org)
    : AbstractSeqItem(org)
{

}


AbstractMultiSeqItem::~AbstractMultiSeqItem()
{

}


std::shared_ptr<AbstractSeq> AbstractMultiSeqItem::abstractSeq()
{
    return abstractMultiSeq();
}


void AbstractMultiSeqItem::doPutProperties(PutPropertyFunction& putProperty)
{
    auto seq = abstractMultiSeq();
    AbstractSeqItem::doPutProperties(putProperty);
    putProperty(_("Num parts"), seq->getNumParts());
}
