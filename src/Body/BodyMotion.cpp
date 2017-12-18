/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "BodyMotion.h"
#include "Body.h"
#include "Link.h"
#include "ZMPSeq.h"
#include <cnoid/Vector3Seq>
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>

using namespace std;
using namespace cnoid;

namespace {
//bool TRACE_FUNCTIONS = false;
}


BodyMotion::BodyMotion()
    : AbstractSeq("CompositeSeq"),
      jointPosSeq_(new MultiValueSeq()),
      linkPosSeq_(new MultiSE3Seq())
{
    setSeqContentName("BodyMotion");
    jointPosSeq_->setSeqContentName("JointDisplacement");
    linkPosSeq_->setSeqContentName("LinkPosition");
}


BodyMotion::BodyMotion(const BodyMotion& org)
    : AbstractSeq(org),
      jointPosSeq_(new MultiValueSeq(*org.jointPosSeq_)),
      linkPosSeq_(new MultiSE3Seq(*org.linkPosSeq_))
{
    for(ExtraSeqMap::const_iterator p = org.extraSeqs.begin(); p != org.extraSeqs.end(); ++p){
        extraSeqs.insert(ExtraSeqMap::value_type(p->first, p->second->cloneSeq()));
    }
}


BodyMotion& BodyMotion::operator=(const BodyMotion& rhs)
{
    if(this != &rhs){
        AbstractSeq::operator=(rhs);
    }
    *jointPosSeq_ = *rhs.jointPosSeq_;
    *linkPosSeq_ = *rhs.linkPosSeq_;

    //! \todo do copy instead of replacing the pointers to the cloned ones
    extraSeqs.clear();
    for(ExtraSeqMap::const_iterator p = rhs.extraSeqs.begin(); p != rhs.extraSeqs.end(); ++p){
        extraSeqs.insert(ExtraSeqMap::value_type(p->first, p->second->cloneSeq()));
    }
    sigExtraSeqsChanged_();

    return *this;
}


AbstractSeqPtr BodyMotion::cloneSeq() const
{
    return std::make_shared<BodyMotion>(*this);
}


/*
  This function sets the number of joints
*/
void BodyMotion::setNumJoints(int numJoints, bool clearNewElements)
{
    jointPosSeq_->setNumParts(numJoints, clearNewElements);
}


double BodyMotion::getFrameRate() const
{
    return frameRate();
}


void BodyMotion::setFrameRate(double frameRate)
{
    jointPosSeq_->setFrameRate(frameRate);
    linkPosSeq_->setFrameRate(frameRate);

    for(ExtraSeqMap::iterator p = extraSeqs.begin(); p != extraSeqs.end(); ++p){
        p->second->setFrameRate(frameRate);
    }
}


int BodyMotion::getNumFrames() const
{
    return numFrames();
}


void BodyMotion::setNumFrames(int n, bool clearNewArea)
{
    jointPosSeq_->setNumFrames(n, clearNewArea);
    linkPosSeq_->setNumFrames(n, clearNewArea);

    for(ExtraSeqMap::iterator p = extraSeqs.begin(); p != extraSeqs.end(); ++p){
        p->second->setNumFrames(n, clearNewArea);
    }
}


int BodyMotion::getOffsetTimeFrame() const {
    return linkPosSeq_->offsetTimeFrame();
}


void BodyMotion::setDimension(int numFrames, int numJoints, int numLinks, bool clearNewArea)
{
    jointPosSeq_->setDimension(numFrames, numJoints, clearNewArea);
    linkPosSeq_->setDimension(numFrames, numLinks, clearNewArea);

    for(ExtraSeqMap::iterator p = extraSeqs.begin(); p != extraSeqs.end(); ++p){
        p->second->setNumFrames(numFrames, clearNewArea);
    }
}


void BodyMotion::setExtraSeq(AbstractSeqPtr seq)
{
    extraSeqs[seq->seqContentName()] = seq;
    sigExtraSeqsChanged_();
}


bool BodyMotion::loadStandardFormat(const std::string& filename, std::ostream& os)
{
    YAMLReader reader;
    reader.expectRegularMultiListing();
    bool result = false;

    try {
        result = readSeq(*reader.loadDocument(filename)->toMapping(), os);
    } catch(const ValueNode::Exception& ex){
        os << ex.message();
    }

    return result;
}


bool BodyMotion::doReadSeq(const Mapping& archive, std::ostream& os)
{
    setDimension(0, 1, 1);

    bool loaded = false;
    
    try {
        double version;
        if(!archive.read("formatVersion", version)){
            version = 1.0;
        }
        const char* type;
        const char* jointContent;
        std::function<string(Mapping* mapping)> readContent;
        if(version >= 2.0){
            type = "CompositeSeq";
            jointContent = "JointDisplacement";
            readContent = [](Mapping* mapping){
                string content;
                mapping->read("content", content);
                return content;
            };
        } else {
            type = "BodyMotion";
            jointContent = "JointPosition";
            readContent = [](Mapping* mapping){
                string content;
                if(!mapping->read("content", content)){
                    mapping->read("purpose", content);
                }
                return content;
            };
        }

        if(archive["type"].toString() == type){

            const Listing& components = *archive["components"].toListing();
        
            for(int i=0; i < components.size(); ++i){
                MappingPtr component = components[i].toMapping()->cloneMapping();

                if(!component->find("formatVersion")->isValid()){
                    component->write("formatVersion", version);
                }
                
                const string type = component->read<string>("type");
                string content = readContent(component);

                if(type == "MultiValueSeq" && content == jointContent){
                    loaded = jointPosSeq_->readSeq(*component, os);
                    if(!loaded){
                        break;
                    }
                } else if((type == "MultiAffine3Seq" || type == "MultiSe3Seq" || type == "MultiSE3Seq")
                          && content == "LinkPosition"){
                    loaded = linkPosSeq_->readSeq(*component, os);
                    if(!loaded){
                        break;
                    }
                } else if(type == "Vector3Seq") {
                    if(content == "ZMP" || content == "RelativeZMP" || content == "RelativeZmp"){
                        auto zmpSeq = getOrCreateExtraSeq<ZMPSeq>("ZMP");
                        loaded = zmpSeq->readSeq(*component, os);
                        if(!loaded){
                            break;
                        }
                        zmpSeq->setRootRelative(content != "ZMP");
                    } else {
                        //----------- user defined Vector3 data --------- 
                        auto userVec3 = getOrCreateExtraSeq<Vector3Seq>(content);
                        loaded = userVec3->readSeq(*component, os);
                        if(!loaded){
                            break;
                        }
                    }
                }
            }
        }
    } catch(const ValueNode::Exception& ex){
        os << ex.message();
        loaded = false;
    }

    if(!loaded){
        setDimension(0, 1, 1);

    }
    
    return loaded;
}


bool BodyMotion::doWriteSeq(YAMLWriter& writer)
{
    double version = writer.info("formatVersion", 2.0);
    bool isVersion1 = version >= 1.0 && version < 2.0;

    writer.setInfo("formatVersion", version);

    writer.setDoubleFormat("%.9g");

    if(isVersion1){
        writer.putComment("Body motion data set format version 1.0 defined by Choreonoid\n");
        writer.putKeyValue("type", "BodyMotion");
    } else {
        AbstractSeq::doWriteSeq(writer);
    }
        
    writer.putKey("components");
    writer.setInfo("isComponent", true);

    writer.startListing();

    if(jointPosSeq_->numFrames() > 0){
        string orgContentName;
        if(isVersion1){
            orgContentName = jointPosSeq_->seqContentName();
            jointPosSeq_->setSeqContentName("JointPosition");
        }
        bool result = jointPosSeq_->writeSeq(writer);
        if(isVersion1){
            jointPosSeq_->setSeqContentName(orgContentName);
        }
        if(!result){
            return false;
        }
    }
    if(linkPosSeq_->numFrames() > 0){
        if(!linkPosSeq_->writeSeq(writer)){
            return false;
        }
    }

    for(ExtraSeqMap::iterator p = extraSeqs.begin(); p != extraSeqs.end(); ++p){
        AbstractSeqPtr& seq = p->second;
        if(!seq->writeSeq(writer)){
            return false;
        }
    }

    writer.endListing();

    return true;
}


bool BodyMotion::saveAsStandardFormat(const std::string& filename, std::ostream& os)
{
    return saveAsStandardFormat(filename, 0.0 /* latest */, os);
}
        

bool BodyMotion::saveAsStandardFormat(const std::string& filename, double version, std::ostream& os)
{
    YAMLWriter writer(filename);
    if(version > 0.0){
        writer.setInfo("formatVersion", version);
    }

    writer.setMessageSink(os);

    return writeSeq(writer);
}


void BodyMotion::clearExtraSeq(const std::string& contentName)
{
    if(extraSeqs.erase(contentName) > 0){
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
    BodyMotion& motion = frame.motion();
    int numJoints = std::min(body.numJoints(), motion.numJoints());
    MultiValueSeq::Frame q = motion.jointPosSeq()->frame(frame.frame());
    for(int i=0; i < numJoints; ++i){
        q[i] = body.joint(i)->q();
    }
    int numLinks =  std::min(body.numLinks(), motion.numLinks());
    MultiSE3Seq::Frame p = motion.linkPosSeq()->frame(frame.frame());
    for(int i=0; i < numLinks; ++i){
        const Link* link = body.link(i);
        p[i].set(link->p(), link->R());
    }
}
    

template<class FrameType>
static void copyFrameToBodyState(FrameType& frame, const Body& body)
{
    const BodyMotion& motion = frame.motion();
    int numJoints = std::min(body.numJoints(), motion.numJoints());
    const MultiValueSeq::Frame q = motion.jointPosSeq()->frame(frame.frame());
    for(int i=0; i < numJoints; ++i){
        body.joint(i)->q() = q[i];
    }
    int numLinks =  std::min(body.numLinks(), motion.numLinks());
    const MultiSE3Seq::Frame p = motion.linkPosSeq()->frame(frame.frame());
    for(int i=0; i < numLinks; ++i){
        Link* link = body.link(i);
        link->p() = p[i].translation();
        link->R() = p[i].rotation().toRotationMatrix();
    }
}


namespace cnoid {

BodyMotion::Frame operator<<(BodyMotion::Frame frame, const Body& body)
{
    copyBodyStateToFrame(body, frame);
    return frame;
}

const Body& operator>>(const Body& body, BodyMotion::Frame frame)
{
    copyBodyStateToFrame(body, frame);
    return body;
}

BodyMotion::Frame operator>>(BodyMotion::Frame frame, const Body& body)
{
    copyFrameToBodyState(frame, body);
    return frame;
}

BodyMotion::ConstFrame operator>>(BodyMotion::ConstFrame frame, Body& body)
{
    copyFrameToBodyState(frame, body);
    return frame;
}

Body& operator<<(Body& body, BodyMotion::Frame frame)
{
    copyFrameToBodyState(frame, body);
    return body;
}

Body& operator<<(Body& body, BodyMotion::ConstFrame frame)
{
    copyFrameToBodyState(frame, body);
    return body;
}

}
