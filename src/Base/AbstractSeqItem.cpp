/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "AbstractSeqItem.h"
#include "Archive.h"
#include "PutPropertyFunction.h"
#include "gettext.h"

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
