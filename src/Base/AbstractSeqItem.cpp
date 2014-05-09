/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "AbstractSeqItem.h"
#include "Archive.h"
#include "PutPropertyFunction.h"
#include <boost/bind.hpp>
#include "gettext.h"

using namespace boost;
using namespace cnoid;


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
    putProperty(_("Number of frames"), seq->getNumFrames(), bind(setPropertyNumFrames, this, _1));
    putProperty(_("Time length"), seq->getTimeLength(), bind(setPropertyTimeLength, this, _1));
    putProperty.decimals(3)(_("Time step"), seq->getTimeStep());
}


bool AbstractSeqItem::store(Archive& archive)
{
    if(overwrite()){
        archive.writeRelocatablePath("filename", filePath());
        archive.write("format", fileFormat());
        return true;
    }
    return false;
}


bool AbstractSeqItem::restore(const Archive& archive)
{
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
