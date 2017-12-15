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
    : AbstractMultiSeq("BodyMotion"),
      jointPosSeq_(new MultiValueSeq()),
      linkPosSeq_(new MultiSE3Seq())
{
    jointPosSeq_->setSeqContentName("JointPosition");
    linkPosSeq_->setSeqContentName("LinkPosition");
}


BodyMotion::BodyMotion(const BodyMotion& org)
    : AbstractMultiSeq(org),
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
        AbstractMultiSeq::operator=(rhs);
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
void BodyMotion::setNumParts(int numParts, bool clearNewElements)
{
    jointPosSeq_->setNumParts(numParts, clearNewElements);
}


/**
   This function returns the number of joints
*/
int BodyMotion::getNumParts() const
{
    return jointPosSeq_->numParts();
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


void BodyMotion::setDimension(int numFrames, int numJoints, bool clearNewArea)
{
    jointPosSeq_->setDimension(numFrames, numJoints, clearNewArea);
    linkPosSeq_->setNumFrames(numFrames, clearNewArea);

    for(ExtraSeqMap::iterator p = extraSeqs.begin(); p != extraSeqs.end(); ++p){
        p->second->setNumFrames(numFrames, clearNewArea);
    }
}


void BodyMotion::setExtraSeq(AbstractSeqPtr seq)
{
    extraSeqs[seq->seqContentName()] = seq;
    sigExtraSeqsChanged_();
}


bool BodyMotion::loadStandardYAMLformat(const std::string& filename, std::ostream& os)
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


/*
bool BodyMotion::read(const Mapping& archive)
{
    return readSeq(archive);
}


bool BodyMotion::write(YAMLWriter& writer)
{
    return writeSeq(writer);
}
*/


bool BodyMotion::doReadSeq(const Mapping& archive, std::ostream& os)
{
    setDimension(0, 1, 1);

    bool loaded = false;
    
    try {
        if(archive["type"].toString() == "BodyMotion"){

            const Listing& components = *archive["components"].toListing();
        
            for(int i=0; i < components.size(); ++i){
                const Mapping& component = *components[i].toMapping();
                const string& type = component["type"];
                string content;
                if(!component.read("content", content)){
                    component.read("purpose", content);
                }
                if(type == "MultiValueSeq" && content == "JointPosition"){
                    loaded = jointPosSeq_->readSeq(component, os);
                    if(!loaded){
                        break;
                    }
                } else if((type == "MultiAffine3Seq" || type == "MultiSe3Seq" || type == "MultiSE3Seq")
                          && content == "LinkPosition"){
                    loaded = linkPosSeq_->readSeq(component, os);
                    if(!loaded){
                        break;
                    }
                } else if(type == "Vector3Seq") {
                    if(content == "ZMP" || content == "RelativeZMP" || content == "RelativeZmp"){
                        auto zmpSeq = getOrCreateExtraSeq<ZMPSeq>("ZMP");
                        loaded = zmpSeq->readSeq(component, os);
                        if(!loaded){
                            break;
                        }
                        zmpSeq->setRootRelative(content != "ZMP");
                    } else {
                        //----------- user defined Vector3 data --------- 
                        auto userVec3 = getOrCreateExtraSeq<Vector3Seq>(content);
                        loaded = userVec3->readSeq(component, os);
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
    writer.putKey("components");

    writer.startListing();

    if(jointPosSeq_->numFrames() > 0){
        if(!jointPosSeq_->writeSeq(writer)){
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


bool BodyMotion::saveAsStandardYAMLformat(const std::string& filename, std::ostream& os)
{
    YAMLWriter writer(filename);

    writer.setMessageSink(os);
    writer.setDoubleFormat("%.9g");
    writer.putComment("Body motion data set format version 1.0 defined by Choreonoid\n");

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
