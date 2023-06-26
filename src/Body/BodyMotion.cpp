#include "BodyMotion.h"
#include "Body.h"
#include "Link.h"
#include "ZMPSeq.h"
#include <cnoid/Vector3Seq>
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

static const string linkPosSeqKey_("MultiLinkPositionSeq");
static const string jointPosSeqKey_("MultiJointDisplacementSeq");

}


BodyMotion::BodyMotion()
    : AbstractSeq("CompositeSeq"),
      positionSeq_(new BodyPositionSeq)
{
    setSeqContentName("BodyMotion");
}


BodyMotion::BodyMotion(const BodyMotion& org)
    : AbstractSeq(org),
      positionSeq_(new BodyPositionSeq(*org.positionSeq_))
{
    for(auto& kv : org.extraSeqs){
        extraSeqs.insert(ExtraSeqMap::value_type(kv.first, kv.second->cloneSeq()));
    }
}


BodyMotion& BodyMotion::operator=(const BodyMotion& rhs)
{
    if(this != &rhs){
        AbstractSeq::operator=(rhs);
    }

    *positionSeq_ = *rhs.positionSeq_;

    extraSeqs.clear();
    for(auto& kv : rhs.extraSeqs){
        extraSeqs.insert(ExtraSeqMap::value_type(kv.first, kv.second->cloneSeq()));
    }
    sigExtraSeqsChanged_();

    return *this;
}


std::shared_ptr<AbstractSeq> BodyMotion::cloneSeq() const
{
    return std::make_shared<BodyMotion>(*this);
}


double BodyMotion::getFrameRate() const
{
    return frameRate();
}


void BodyMotion::setFrameRate(double frameRate)
{
    positionSeq_->setFrameRate(frameRate);

    for(auto& kv : extraSeqs){
        kv.second->setFrameRate(frameRate);
    }
}


int BodyMotion::getNumFrames() const
{
    return numFrames();
}


void BodyMotion::setNumFrames(int n, bool fillNewElements)
{
    positionSeq_->setNumFrames(n);

    for(auto& kv : extraSeqs){
        kv.second->setNumFrames(n, fillNewElements);
    }
}


double BodyMotion::getOffsetTime() const
{
    return positionSeq_->offsetTime();
}


void BodyMotion::setOffsetTime(double time)
{
    positionSeq_->setOffsetTime(time);

    for(auto& kv : extraSeqs){
        kv.second->setOffsetTime(time);
    }
}


std::shared_ptr<MultiSE3Seq> BodyMotion::getOrCreateLinkPosSeq()
{
    return getOrCreateExtraSeq<MultiSE3Seq>(
        linkPosSeqKey_,
        [this](MultiSE3Seq& seq){
            seq.setSeqContentName(linkPosSeqKey_);
            seq.setNumParts(positionSeq_->numLinkPositionsHint());
        });
}


std::shared_ptr<MultiValueSeq> BodyMotion::getOrCreateJointPosSeq()
{
    return getOrCreateExtraSeq<MultiValueSeq>(
        jointPosSeqKey_,
        [this](MultiValueSeq& seq){
            seq.setSeqContentName(jointPosSeqKey_);
            seq.setNumParts(positionSeq_->numJointDisplacementsHint());
        });
}


void BodyMotion::setDimension(int numFrames, int numJoints, int numLinks, bool fillNewElements)
{
    positionSeq_->setNumFrames(numFrames);
    positionSeq_->setNumJointDisplacementsHint(numJoints);
    positionSeq_->setNumLinkPositionsHint(numLinks);

    for(auto& kv : extraSeqs){
        auto& seq = kv.second;
        bool done = false;
        if(auto multiSeq = dynamic_pointer_cast<AbstractMultiSeq>(seq)){
            if(multiSeq->seqContentName() == linkPosSeqKey_){
                multiSeq->setDimension(numFrames, numLinks, fillNewElements);
                done = true;
            } else if(multiSeq->seqContentName() == jointPosSeqKey_){
                multiSeq->setDimension(numFrames, numJoints, fillNewElements);
                done = true;
            }
        }
        if(!done){
            seq->setNumFrames(numFrames, fillNewElements);
        }
    }
}


void BodyMotion::setNumJoints(int numJoints, bool fillNewElements)
{
    positionSeq_->setNumJointDisplacementsHint(numJoints);
    if(auto seq = extraSeq<MultiValueSeq>(jointPosSeqKey_)){
        seq->setNumParts(numJoints, fillNewElements);
    }
}


std::shared_ptr<MultiSE3Seq> BodyMotion::linkPosSeq()
{
    return getOrCreateLinkPosSeq();
}


std::shared_ptr<const MultiSE3Seq> BodyMotion::linkPosSeq() const
{
    return const_cast<BodyMotion*>(this)->getOrCreateLinkPosSeq();
}


std::shared_ptr<MultiValueSeq> BodyMotion::jointPosSeq()
{
    return getOrCreateJointPosSeq();
}


std::shared_ptr<const MultiValueSeq> BodyMotion::jointPosSeq() const
{
    return const_cast<BodyMotion*>(this)->getOrCreateJointPosSeq();
}


const std::string& BodyMotion::linkPosSeqKey()
{
    return linkPosSeqKey_;
}


const std::string& BodyMotion::jointPosSeqKey()
{
    return jointPosSeqKey_;
}


void BodyMotion::updateLinkPosSeqWithBodyPositionSeq()
{
    auto lseq = getOrCreateLinkPosSeq();
    const int numLinks = positionSeq_->numLinkPositionsHint();
    if(numLinks > 0){
        lseq->setNumParts(numLinks);
        const int n = numFrames();
        for(int i=0; i < n; ++i){
            auto& pframe = positionSeq_->frame(i);
            auto lframe = lseq->frame(i);
            int linkIndex = 0;
            int m = std::min(numLinks, pframe.numLinkPositions());
            while(linkIndex < m){
                auto linkPosition = pframe.linkPosition(linkIndex);
                lframe[linkIndex].set(linkPosition.translation(), linkPosition.rotation());
                ++linkIndex;
            }
            while(linkIndex < numLinks){
                lframe[linkIndex].clear();
            }
        }
    }
}


void BodyMotion::updateJointPosSeqWithBodyPositionSeq()
{
    auto jseq = getOrCreateJointPosSeq();
    const int numJoints = positionSeq_->numJointDisplacementsHint();
    if(numJoints > 0){
        jseq->setNumParts(numJoints);
        const int n = numFrames();
        for(int i=0; i < n; ++i){
            auto& pframe = positionSeq_->frame(i);
            int jointIndex = 0;
            auto jframe = jseq->frame(i);
            auto displacements = pframe.jointDisplacements();
            int m = std::min(numJoints, pframe.numJointDisplacements());
            while(jointIndex < m){
                jframe[jointIndex] = displacements[jointIndex];
                ++jointIndex;
            }
            while(jointIndex < numJoints){
                jframe[jointIndex] = 0.0;
            }
        }
    }
}


void BodyMotion::updateLinkPosSeqAndJointPosSeqWithBodyPositionSeq()
{
    updateLinkPosSeqWithBodyPositionSeq();
    updateJointPosSeqWithBodyPositionSeq();
}


void BodyMotion::updateBodyPositionSeqWithLinkPosSeqAndJointPosSeq()
{
    shared_ptr<AbstractSeq> srcSeq;

    int numLinkPosSeqParts = 0;
    auto lseq = extraSeq<MultiSE3Seq>(linkPosSeqKey_);
    if(lseq){
        numLinkPosSeqParts = lseq->numParts();
        srcSeq = lseq;
    }
    
    int numJointPosSeqParts = 0;
    auto jseq = extraSeq<MultiValueSeq>(jointPosSeqKey_);
    if(jseq){
        numJointPosSeqParts = jseq->numParts();
        if(!srcSeq){
            srcSeq = jseq;
        }
    }

    int numSrcFrames = srcSeq->getNumFrames();
    positionSeq_->setNumLinkPositionsHint(numLinkPosSeqParts);
    positionSeq_->setNumJointDisplacementsHint(numJointPosSeqParts);
    positionSeq_->setNumFrames(numSrcFrames);
    positionSeq_->setFrameRate(srcSeq->getFrameRate());
    positionSeq_->setOffsetTime(srcSeq->getOffsetTime());
    
    for(int i=0; i < numSrcFrames; ++i){
        auto& pframe = positionSeq_->allocateFrame(i);

        if(numLinkPosSeqParts > 0){
            auto lframe = lseq->frame(i);
            for(int j=0; j < numLinkPosSeqParts; ++j){
                auto linkPosition = pframe.linkPosition(j);
                linkPosition.set(lframe[j]);
            }
        }
        if(numJointPosSeqParts > 0){
            auto jframe = jseq->frame(i);
            auto displacements = pframe.jointDisplacements();
            for(int j=0; j < numJointPosSeqParts; ++j){
                displacements[j] = jframe[j];
            }
        }
    }
}


void BodyMotion::setExtraSeq(const std::string& name, std::shared_ptr<AbstractSeq> seq)
{
    extraSeqs[name] = seq;
    sigExtraSeqsChanged_();
}


void BodyMotion::clearExtraSeq(const std::string& name)
{
    if(extraSeqs.erase(name) > 0){
        sigExtraSeqsChanged_();
    }
}


static BodyMotion& emptyMotion()
{
    static BodyMotion motion;
    return motion;
}


BodyMotion::Frame::Frame()
    : motion_(emptyMotion()),
      frame_(-1)
{

}


BodyMotion::ConstFrame::ConstFrame()
    : motion_(emptyMotion()),
      frame_(-1)
{

}

    
static void copyBodyStateToFrame(const Body& body, BodyMotion::Frame& frame)
{
    body >> frame.motion().positionSeq()->allocateFrame(frame.frame());
}
    

template<class FrameType>
static void copyFrameToBodyState(FrameType& frame, Body& body)
{
    frame.motion().positionSeq()->frame(frame.frame()) >> body;
}


BodyMotion::Frame cnoid::operator<<(BodyMotion::Frame frame, const Body& body)
{
    copyBodyStateToFrame(body, frame);
    return frame;
}


BodyMotion::Frame cnoid::operator>>(BodyMotion::Frame frame, Body& body)
{
    copyFrameToBodyState(frame, body);
    return frame;
}


BodyMotion::ConstFrame cnoid::operator>>(BodyMotion::ConstFrame frame, Body& body)
{
    copyFrameToBodyState(frame, body);
    return frame;
}


Body& cnoid::operator<<(Body& body, BodyMotion::Frame frame)
{
    copyFrameToBodyState(frame, body);
    return body;
}


Body& cnoid::operator<<(Body& body, BodyMotion::ConstFrame frame)
{
    copyFrameToBodyState(frame, body);
    return body;
}


const Body& cnoid::operator>>(const Body& body, BodyMotion::Frame frame)
{
    copyBodyStateToFrame(body, frame);
    return body;
}


bool BodyMotion::load(const std::string& filename, std::ostream& os)
{
    YAMLReader reader;
    reader.expectRegularMultiListing();
    bool result = false;

    try {
        result = readSeq(reader.loadDocument(filename)->toMapping(), os);
    } catch(const ValueNode::Exception& ex){
        os << ex.message();
    }

    return result;
}


bool BodyMotion::doReadSeq(const Mapping* archive, std::ostream& os)
{
    setDimension(0, 1, 1);

    bool loaded = false;
    
    double version;
    if(!archive->read("formatVersion", version)){
        version = 1.0;
    }
    if(version >= 4.0){
        os << format(_("Format version {} is not supported"), version) << endl;
        return false;
    }
    
    string type;
    const char* jointContent;
    const char* linkContent;
    std::function<string(Mapping* mapping)> readContent;
    if(version >= 2.0){
        type = seqType();
        if(version >= 3.0){
            jointContent = "MultiJointDisplacementSeq";
            linkContent = "MultiLinkPositionSeq";
        } else {
            jointContent = "JointDisplacement";
            linkContent = "LinkPosition";
        }
        readContent = [](Mapping* mapping){
            string content;
            mapping->read("content", content);
            return content;
        };
    } else {
        type = "BodyMotion";
        jointContent = "JointPosition";
        linkContent = "LinkPosition";
        readContent = [](Mapping* mapping){
            string content;
            if(!mapping->read("content", content)){
                mapping->read("purpose", content);
            }
            return content;
        };
    }
    
    if(archive->get<string>("type") == type){
        
        const Listing& components = *(*archive)["components"].toListing();
        
        for(int i=0; i < components.size(); ++i){

            // Merge the parameters of the parent node into the child (component) node
            MappingPtr component = components[i].toMapping()->cloneMapping();
            component->insert(archive);

            const ValueNode& typeNode = (*component)["type"];
            const string type = typeNode.toString();
            string content = readContent(component);
            
            if((type == "MultiSE3Seq" || (version < 2.0 && (type == "MultiSe3Seq" || type == "MultiAffine3Seq")))){
                if(content == linkContent){
                    loaded = linkPosSeq()->readSeq(component, os);
                    if(!loaded) break;
                    linkPosSeq()->setSeqContentName("MultiLinkPositionSeq");
                } else {
                    os << format(_("Unknown content \"{0}\" of type \"{1}\"."), content, type) << endl;
                }
            } else if(type == "MultiValueSeq"){
                if(content == jointContent){
                    auto jseq = jointPosSeq();
                    loaded = jseq->readSeq(component, os);
                    if(!loaded) break;
                    jseq->setSeqContentName("MultiJointDisplacementSeq");
                } else {
                    os << format(_("Unknown content \"{0}\" of type \"{1}\"."), content, type) << endl;
                }
            } else if(type == "Vector3Seq") {
                if((version >= 3.0 && content == "ZMPSeq") ||
                   (version < 3.0 && content == "ZMP") ||
                   ((version < 2.0) && (content == "RelativeZMP" || content == "RelativeZmp"))){
                    auto zmpSeq = getOrCreateZMPSeq(*this);
                    loaded = zmpSeq->readSeq(component, os);
                    if(!loaded){
                        break;
                    }
                    if(version < 2.0){
                        zmpSeq->setRootRelative(content != "ZMP");
                    }
                } else {
                    //----------- user defined Vector3 data --------- 
                    auto userVec3 = getOrCreateExtraSeq<Vector3Seq>(content);
                    loaded = userVec3->readSeq(component, os);
                    if(!loaded){
                        break;
                    }
                }
            } else {
                os << format(_("Unknown type \"{}\"."), type) << endl;
            }
        }
    }
    
    if(loaded){
        updateBodyPositionSeqWithLinkPosSeqAndJointPosSeq();
    } else {
        setDimension(0, 1, 1);
    }
    clearExtraSeq(linkPosSeqKey_);
    clearExtraSeq(jointPosSeqKey_);
    
    return loaded;
}


bool BodyMotion::doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback)
{
    bool doClearLinkPosSeq = extraSeqs.find(linkPosSeqKey_) == extraSeqs.end();
    bool doClearJointPosSeq = extraSeqs.find(jointPosSeqKey_) == extraSeqs.end();
    
    updateLinkPosSeqAndJointPosSeqWithBodyPositionSeq();
    
    double version = writer.getOrCreateInfo("formatVersion", 3.0);

    if(version < 2.0){
        writer.putComment("Body motion data set format version 1.0 defined by Choreonoid\n");
        setSeqType("BodyMotion");
    }

    bool result = AbstractSeq::doWriteSeq(
        writer,
        [&](){
            writer.setDoubleFormat("%.9g");

            if(additionalPartCallback) additionalPartCallback();

            writer.putKey("components");
            writer.setInfo("isComponent", true);

            writer.startListing();

            auto lseq = linkPosSeq();
            if(lseq->numFrames() > 0 && lseq->numParts() > 0){
                lseq->writeSeq(writer);
            }

            auto jseq = jointPosSeq();
            if(jseq->numFrames() > 0 && jseq->numParts() > 0){
                string orgContentName;
                if(version < 3.0){
                    orgContentName = jseq->seqContentName();
                    if(version >= 2.0){
                        jseq->setSeqContentName("JointDisplacement");
                    } else {
                        jseq->setSeqContentName("JointPosition");
                    }
                }
                jseq->writeSeq(writer);
                if(version < 3.0){
                    jseq->setSeqContentName(orgContentName);
                }
            }
            
            for(auto& kv : extraSeqs){
                if(kv.first != linkPosSeqKey_ && kv.first != jointPosSeqKey_){
                    kv.second->writeSeq(writer);
                }
            }
            
            writer.endListing();
        });

    if(version < 2.0){
        setSeqType("CompositeSeq");
    }

    if(doClearLinkPosSeq){
        clearExtraSeq(linkPosSeqKey_);
    }
    if(doClearJointPosSeq){
        clearExtraSeq(jointPosSeqKey_);
    }

    return result;
}


bool BodyMotion::save(const std::string& filename, std::ostream& os)
{
    return save(filename, 0.0 /* latest */, os);
}
        

bool BodyMotion::save(const std::string& filename, double version, std::ostream& os)
{
    YAMLWriter writer(filename);
    if(version > 0.0){
        writer.setInfo("formatVersion", version);
    }

    writer.setMessageSink(os);

    return writeSeq(writer);
}
