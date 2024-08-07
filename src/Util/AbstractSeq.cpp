#include "AbstractSeq.h"
#include "YAMLWriter.h"
#include "Format.h"
#include <ostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

string dummySeqMessage;

}


AbstractSeq::AbstractSeq(const char* seqType)
    : seqType_(seqType)
{

}


AbstractSeq::AbstractSeq(const AbstractSeq& org)
    : seqType_(org.seqType_),
      contentName_(org.contentName_)
{

}


AbstractSeq& AbstractSeq::operator=(const AbstractSeq& rhs)
{
    seqType_ = rhs.seqType_;
    contentName_ = rhs.contentName_;
    return *this;
}


void AbstractSeq::copySeqProperties(const AbstractSeq& source)
{
    seqType_ = source.seqType_;
    contentName_ = source.contentName_;
}


AbstractSeq::~AbstractSeq()
{

}


void AbstractSeq::setSeqType(const std::string& type)
{
    seqType_ = type;
}


void AbstractSeq::setSeqContentName(const std::string& name)
{
    this->contentName_ = name;
}


double AbstractSeq::defaultFrameRate()
{
    return 100.0;
}


double AbstractSeq::getTimeStep() const
{
    double r = getFrameRate();
    return (r > 0.0) ? 1.0 / r : 0.0;
}
    

void AbstractSeq::setTimeStep(double timeStep)
{
    if(timeStep > 0.0){
        setFrameRate(1.0 / timeStep);
    } else {
        setFrameRate(0.0);
    }
}


double AbstractSeq::getTimeOfFrame(int frame) const
{
    double r = getFrameRate();
    return (r > 0.0) ? (frame / r) + getOffsetTime() : getOffsetTime();
}


int AbstractSeq::getFrameOfTime(double time) const
{
    return static_cast<int>((time - getOffsetTime()) * getFrameRate());
}


int AbstractSeq::getOffsetTimeFrame() const
{
    return static_cast<int>(getOffsetTime() * getFrameRate());
}


void AbstractSeq::setOffsetTimeFrame(int frame)
{
    setOffsetTime(frame / getFrameRate());
}


double AbstractSeq::getTimeLength() const
{
    double r = getFrameRate();
    return (r > 0.0) ? (getNumFrames() / r) : 0.0;
}


bool AbstractSeq::readSeq(const Mapping* archive, std::ostream& os)
{
    bool result = false;
    
    try {
        result = doReadSeq(archive, os);
    }
    catch (ValueNode::Exception& ex) {
        os << ex.message();
    }

    return result;
}


bool AbstractSeq::doReadSeq(const Mapping*, std::ostream& os)
{
    os << formatR(_("The function to read {} is not implemented."), seqType()) << endl;
    return false;
}


bool AbstractSeq::writeSeq(YAMLWriter& writer)
{
    return doWriteSeq(writer, nullptr);
}


bool AbstractSeq::doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback)
{
    if(seqType_.empty()){
        if(contentName_.empty()){
            writer.putMessage(_("The type of the sequence to write is unknown.\n"));
        } else {
            writer.putMessage(formatR(_("The type of the {} sequence to write is unknown.\n"), contentName_));
        }
        return false;
    }

    const double frameRate = getFrameRate();
    if(frameRate <= 0.0){
        writer.putMessage(
            formatR(_("Frame rate {0} of {1} is invalid.\n"),
                    frameRate, (contentName_.empty() ? seqType_ : contentName_)));
        return false;
    }

    writer.startMapping();
    
    writer.putKeyValue("type", seqType_);
    if(!contentName_.empty()){
        writer.putKeyValue("content", contentName_);
    }

    bool doWriteFormatVersion = !writer.info("is_component", false);
    double version = writer.info("format_version", 4.0);
    if(version >= 4.0){
        if(doWriteFormatVersion){
            writer.putKeyValue("format_version", version);
        }
        writer.putKeyValue("frame_rate", frameRate);
        writer.putKeyValue("num_frames", getNumFrames());
    } else {
        if(doWriteFormatVersion){
            writer.putKeyValue("formatVersion", version);
        }
        writer.putKeyValue("frameRate", frameRate);
        writer.putKeyValue("numFrames", getNumFrames());
    }

    if(additionalPartCallback) additionalPartCallback();

    writer.endMapping();
    
    return true;
}


const std::string& AbstractSeq::seqMessage() const
{
    return dummySeqMessage;
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


bool AbstractMultiSeq::doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback)
{
    return AbstractSeq::doWriteSeq(
        writer,
        [&](){
            double version = writer.info("format_version", 0.0);
            writer.putKeyValue(version >= 4.0 ? "num_parts" : "numParts", getNumParts());
            if(additionalPartCallback) additionalPartCallback();
        });
}


std::vector<std::string> AbstractMultiSeq::readSeqPartLabels(const Mapping* archive)
{
    vector<string> labelStrings;
    const Listing& labels = *archive->findListing({ "part_labels", "partLabels"});
    if(labels.isValid()){
        int n = labels.size();
        labelStrings.reserve(n);
        for(int i=0; i < n; ++i){
            labelStrings.push_back(labels[i].toString());
        }
    }
    return labelStrings;
}


bool AbstractMultiSeq::writeSeqPartLabels(YAMLWriter& writer)
{
    double version = writer.info("format_version",  0.0);
    writer.putKey(version >= 4.0 ? "part_labels" : "partLabels");
    writer.startFlowStyleListing();
    int n = getNumParts();
    for(int i=0; i < n; ++i){
        writer.putDoubleQuotedString(partLabel(i));
    }
    writer.endListing();
    return true;
}
