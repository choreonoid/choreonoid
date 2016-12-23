/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "AbstractSeqItem.h"
#include "Archive.h"
#include "PutPropertyFunction.h"
#include "gettext.h"

using namespace cnoid;
using namespace std::placeholders;

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


static bool setOffsetTime(AbstractSeqItem* item, double offset)
{
    return item->abstractSeq()->setOffsetTime(offset);
}


static bool setPropertyNumFrames(AbstractSeqItem* item, int numFrames)
{
    if(numFrames >= 0){
        item->abstractSeq()->setNumFrames(numFrames);
        item->suggestFileUpdate();
        return true;
    }
    return false;
}


static bool setPropertyTimeLength(AbstractSeqItem* item, double timeLength)
{
    if(timeLength >= 0){
        item->abstractSeq()->setTimeLength(timeLength);
        item->suggestFileUpdate();
        return true;
    }
    return false;
}


void AbstractSeqItem::doPutProperties(PutPropertyFunction& putProperty)
{
    AbstractSeqPtr seq = abstractSeq();
    putProperty(_("Frame rate"), seq->getFrameRate());
    putProperty(_("Offset time"), seq->getOffsetTime(), std::bind(setOffsetTime, this, _1));
    putProperty(_("Number of frames"), seq->getNumFrames(), std::bind(setPropertyNumFrames, this, _1));
    putProperty(_("Time length"), seq->getTimeLength(), std::bind(setPropertyTimeLength, this, _1));
    putProperty.decimals(3)(_("Time step"), seq->getTimeStep());
}


bool AbstractSeqItem::store(Archive& archive)
{
    if(overwrite()){
        archive.writeRelocatablePath("filename", filePath());
        archive.write("format", fileFormat());
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
    std::string filename, formatId;
    if(archive.readRelocatablePath("filename", filename) && archive.read("format", formatId)){
        if(load(filename, formatId)){
            return true;
        }
    }
    return false;
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


AbstractSeqPtr AbstractMultiSeqItem::abstractSeq()
{
    return abstractMultiSeq();
}


void AbstractMultiSeqItem::doPutProperties(PutPropertyFunction& putProperty)
{
    AbstractMultiSeqPtr seq = abstractMultiSeq();
    AbstractSeqItem::doPutProperties(putProperty);
    putProperty(_("Num parts"), seq->getNumParts());
}
