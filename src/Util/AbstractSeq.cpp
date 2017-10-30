/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "AbstractSeq.h"
#include "YAMLWriter.h"
#include <boost/format.hpp>

using namespace std;
using namespace cnoid;

using boost::format;


AbstractSeq::AbstractSeq(const char* seqType)
    : seqType_(seqType)
{

}


AbstractSeq::AbstractSeq(const AbstractSeq& org)
    : seqType_(org.seqType_),
      content(org.content)
{

}


AbstractSeq& AbstractSeq::operator=(const AbstractSeq& rhs)
{
    seqType_ = rhs.seqType_;
    content = rhs.content;
    return *this;
}


void AbstractSeq::copySeqProperties(const AbstractSeq& source)
{
    seqType_ = source.seqType_;
    content = source.content;
}


AbstractSeq::~AbstractSeq()
{

}


double AbstractSeq::getTimeOfFrame(int frame) const
{
    return (frame + getOffsetTimeFrame()) / getFrameRate();
}


int AbstractSeq::getFrameOfTime(double time) const
{
    return static_cast<int>(time * getFrameRate()) - getOffsetTimeFrame();
}


int AbstractSeq::getOffsetTimeFrame() const
{
    return 0;
}


bool AbstractSeq::setOffsetTimeFrame(int offset)
{
    return (offset == 0);
}


bool AbstractSeq::readSeq(const Mapping& archive)
{
    try {
        if(content.empty()){
            if(!archive.read("content", content)){
                archive.read("purpose", content); // old version
            }
        }
        setFrameRate(archive["frameRate"].toDouble());

        return doReadSeq(archive);

    } catch (ValueNode::Exception& ex) {
        addSeqMessage(ex.message());
        return false;
    }
}


bool AbstractSeq::doReadSeq(const Mapping&)
{
    return true;
}


bool AbstractSeq::checkSeqContent(const Mapping& archive, const std::string contentName, bool throwEx)
{
    string content_;
    if(!archive.read("content", content_)){
        archive.read("purpose", content_); // old version
    }
    bool result = (content_ == contentName);

    if(!result){
        string message;

        if(content_.empty()){
            message =
                str(format("Content of %1% should be \"%2%\" but it is not specified.")
                    % seqType() % contentName);
        } else {
            message =
                str(format("Content \"%1%\" of %2% is different from the required content \"%3%\".")
                    % content_ % seqType() % contentName);
        }
        if(throwEx){
            archive.throwException(message);
        } else {
            addSeqMessage(message);
        }
    }
    return result;
}


bool AbstractSeq::writeSeq(YAMLWriter& writer)
{
    bool result = false;
    
    if(seqType().empty()){
        addSeqMessage("setType() is empty.");

    } else {
        writer.startMapping();
        
        writer.putKeyValue("type", seqType());
        writer.putKeyValue("content", seqContentName());
        writer.putKeyValue("frameRate", getFrameRate());
        writer.putKeyValue("numFrames", getNumFrames());

        result = doWriteSeq(writer);

        writer.endMapping();
    }
    
    return result;
}


bool AbstractSeq::doWriteSeq(YAMLWriter& /* writer */)
{
    return true;
}


AbstractMultiSeq::AbstractMultiSeq(const char* seqType)
    : AbstractSeq(seqType)
{

}


AbstractMultiSeq::AbstractMultiSeq(const AbstractMultiSeq& org)
    : AbstractSeq(org)
{

}


AbstractMultiSeq& AbstractMultiSeq::operator=(const AbstractMultiSeq& rhs)
{
    AbstractSeq::operator=(rhs);
    return *this;
}


void AbstractMultiSeq::copySeqProperties(const AbstractMultiSeq& source)
{
    AbstractSeq::copySeqProperties(source);
}


AbstractMultiSeq::~AbstractMultiSeq()
{

}


int AbstractMultiSeq::partIndex(const std::string& /* partLabel*/) const
{
    return -1;
}


const std::string& AbstractMultiSeq::partLabel(int /* partIndex */) const
{
    static const std::string nolabel;
    return nolabel;
}


bool AbstractMultiSeq::doWriteSeq(YAMLWriter& writer)
{
    writer.putKeyValue("numParts", getNumParts());
    return true;
}


bool AbstractMultiSeq::readSeqPartLabels(const Mapping& archive, SetPartLabelFunction setPartLabel)
{
    const Listing& labels = *archive.findListing("partLabels");
    if(labels.isValid()){
        for(int i=0; i < labels.size(); ++i){
            setPartLabel(labels[i].toString(), i);
        }
        return true;
    } else {
        addSeqMessage("\"partLabels\" is not valid.");
    }
    return false;
}


bool AbstractMultiSeq::writeSeqPartLabels(YAMLWriter& writer)
{
    writer.putKey("partLabels");
    writer.startFlowStyleListing();
    int n = getNumParts();
    for(int i=0; i < n; ++i){
        writer.putDoubleQuotedString(partLabel(i));
    }
    writer.endListing();
    return true;
}
