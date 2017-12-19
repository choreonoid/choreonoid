/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "AbstractSeq.h"
#include "YAMLWriter.h"
#include <boost/format.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;

using boost::format;

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


bool AbstractSeq::readSeq(const Mapping& archive, std::ostream& os)
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


bool AbstractSeq::doReadSeq(const Mapping& archive, std::ostream&)
{
    readSeqContent(archive);

    if(archive.get("hasFrameTime", false)){
        archive.throwException(
            _("Sequence data with frame time cannot be loaded as regular interval sequence data."));
    }
    
    setFrameRate(archive.read<double>("frameRate"));

    return true;
}


bool AbstractSeq::importTimedFrameSeq(const Mapping& archive, std::ostream& os)
{
    bool result = false;
    
    try {
        checkSeqFormatVersion(archive, 2.0);

        if(getFrameRate() <= 0.0){
            os << _("The frame rate for importing timed-frame seq data is not specified.") << endl;
        } else {
            result = doImportTimedFrameSeq(archive, os);
        }
    }
    catch (ValueNode::Exception& ex){
        os << ex.message();
    }

    return result;
}


bool AbstractSeq::doImportTimedFrameSeq(const Mapping& archive, std::ostream& os)
{
    os << format(_("Importing timed-frame seq data into %1% is not supported.")) % seqType() << endl;
    return false;
}


double AbstractSeq::checkSeqFormatVersion(const Mapping& archive, double minVersion, double strictMaxVersion)
{
    auto versionNode = archive.find("formatVersion");
    double version = versionNode->isValid() ? versionNode->toDouble() : 1.0;
    bool isSupported = true;
    if(minVersion > 0.0 && version < minVersion){
        isSupported = false;
    } else if(strictMaxVersion > 0.0 && version >= strictMaxVersion){
        isSupported = false;
    }
    if(!isSupported){
        const ValueNode* node = versionNode->isValid() ? versionNode : &archive;
        node->throwException(str(format(_("Format version %1% is not supported in this operation.")) % version));
    }
    return version;
}    
    

void AbstractSeq::checkSeqType(const Mapping& archive)
{
    auto& typeNode = archive["type"];
    if(typeNode.toString() != seqType()){
        typeNode.throwException(str(format(_("The seq type must be %1%.")) % seqType()));
    }
}


bool AbstractSeq::readSeqContent(const Mapping& archive)
{
    if(!archive.read("content", contentName_)){
        archive.read("purpose", contentName_); // old version
    }
    return !contentName_.empty();
}


bool AbstractSeq::checkSeqContent(const Mapping& archive, const std::string requiredContent, std::ostream& os)
{
    readSeqContent(archive);
    
    if(contentName_ == requiredContent){
        return true;
    }

    if(contentName_.empty()){
        os << format(_("Content of %1% should be \"%2%\" but it is not specified."))
            % seqType() % requiredContent;
    } else {
        os << format(_("Content \"%1%\" of %2% is different from the required content \"%3%\"."))
            % contentName_ % seqType() % requiredContent;
    }
    return false;
}


int AbstractSeq::readNumParts(const Mapping& archive)
{
    auto& node = archive["numParts"];
    const int n = node.toInt();
    if(n < 1){
        node.throwException(_("Invaid number of parts is specified"));
    }
    return n;
}
    

bool AbstractSeq::writeSeq(YAMLWriter& writer)
{
    writer.startMapping();
        
    bool result = doWriteSeq(writer);
    
    writer.endMapping();
    
    return result;
}


bool AbstractSeq::doWriteSeq(YAMLWriter& writer)
{
    if(seqType_.empty()){
        if(contentName_.empty()){
            writer.putMessage(_("The type of the sequence to write is unknown."));
        } else {
            writer.putMessage(str(format(_("The type of the %1% sequence to write is unknown.")) % contentName_));
        }
        return false;
    }

    const double frameRate = getFrameRate();
    if(frameRate <= 0.0){
        writer.putMessage(
            str(format(_("Frame rate %1% of %2% is invalid"))
                % frameRate % (contentName_.empty() ? seqType_ : contentName_)));
        return false;
    }

    writer.putKeyValue("type", seqType_);
    if(!contentName_.empty()){
        writer.putKeyValue("content", contentName_);
    }

    double version = writer.info("formatVersion", 0.0);
    if(version == 0.0 || !writer.info("isComponent", false)){
        writer.putKeyValue("formatVersion", version == 0.0 ? 2.0 : version);
    }
    writer.putKeyValue("frameRate", frameRate);
    writer.putKeyValue("numFrames", getNumFrames());

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


bool AbstractMultiSeq::doWriteSeq(YAMLWriter& writer)
{
    if(AbstractSeq::doWriteSeq(writer)){
        writer.putKeyValue("numParts", getNumParts());
        return true;
    }
    return false;
}


std::vector<std::string> AbstractMultiSeq::readSeqPartLabels(const Mapping& archive)
{
    vector<string> labelStrings;
    const Listing& labels = *archive.findListing("partLabels");
    if(labels.isValid()){
        for(int i=0; i < labels.size(); ++i){
            labelStrings.push_back(labels[i].toString());
        }
    }
    return labelStrings;
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
