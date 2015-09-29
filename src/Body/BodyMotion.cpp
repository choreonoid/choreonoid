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
    return boost::make_shared<BodyMotion>(*this);
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


bool BodyMotion::loadStandardYAMLformat(const std::string& filename)
{
    bool result = false;
    clearSeqMessage();
    YAMLReader reader;
    reader.expectRegularMultiListing();
    
    try {
        result = read(*reader.loadDocument(filename)->toMapping());
    } catch(const ValueNode::Exception& ex){
        addSeqMessage(ex.message());
    }

    return result;
}


bool BodyMotion::read(const Mapping& archive)
{
    setDimension(0, 1, 1);

    bool result = true;
    bool loaded = false;
    ZMPSeqPtr zmpSeq;
    
    try {
        if(archive["type"].toString() != "BodyMotion"){
            result = false;

        } else {
            const Listing& components = *archive["components"].toListing();
        
            for(int i=0; i < components.size(); ++i){
                const Mapping& component = *components[i].toMapping();
                const string& type = component["type"];
                string content;
                if(!component.read("content", content)){
                    component.read("purpose", content);
                }
                if(type == "MultiValueSeq" && content == "JointPosition"){
                    result &= jointPosSeq_->readSeq(component);
                    if(result){
                        loaded = true;
                    } else {
                        addSeqMessage(jointPosSeq_->seqMessage());
                    }
                } else if((type == "MultiAffine3Seq" || type == "MultiSe3Seq" || type == "MultiSE3Seq")
                          && content == "LinkPosition"){
                    result &= linkPosSeq_->readSeq(component);
                    if(result){
                        loaded = true;
                    } else {
                        addSeqMessage(linkPosSeq_->seqMessage());
                    }
                } else if(type == "Vector3Seq") {
                    bool isRelativeZmp = false;
                    if(content == "RelativeZMP" || content == "RelativeZmp"){
                        isRelativeZmp = true;
                    } else if(content != "ZMP"){
                        continue;
                    }
                    zmpSeq = getOrCreateExtraSeq<ZMPSeq>("ZMP");
                    result = zmpSeq->readSeq(component);
                    if(result){
                        if(isRelativeZmp){
                            zmpSeq->setRootRelative(true);
                        }
                    } else {
                        addSeqMessage(zmpSeq->seqMessage());
                    }
                }
                if(!result){
                    break;
                }
            }
        }
    } catch(const ValueNode::Exception& ex){
        addSeqMessage(ex.message());
        result = false;
    }

    if(!result){
        setDimension(0, 1, 1);

    }
    
    return (result && loaded);
}


bool BodyMotion::write(YAMLWriter& writer)
{
    writer.startListing();

    if(jointPosSeq_->numFrames() > 0){
        if(!jointPosSeq_->writeSeq(writer)){
            addSeqMessage(jointPosSeq_->seqMessage());
            return false;
        }
    }
    if(linkPosSeq_->numFrames() > 0){
        if(!linkPosSeq_->writeSeq(writer)){
            addSeqMessage(linkPosSeq_->seqMessage());
            return false;
        }
    }

    for(ExtraSeqMap::iterator p = extraSeqs.begin(); p != extraSeqs.end(); ++p){
        AbstractSeqPtr& seq = p->second;
        if(!seq->writeSeq(writer)){
            addSeqMessage(seq->seqMessage());
            return false;
        }
    }

    writer.endListing();

    return true;
}


bool BodyMotion::saveAsStandardYAMLformat(const std::string& filename)
{
    YAMLWriter writer(filename);
    writer.setDoubleFormat("%.9g");

    writer.putComment("Body motion data set format version 1.0 defined by Choreonoid\n");

    writer.startMapping();

    writer.putKeyValue("type", "BodyMotion");
    writer.putKey("components");

    bool result = write(writer);

    writer.endMapping();

    return result;
}


void BodyMotion::clearExtraSeq(const std::string& contentName)
{
    if(extraSeqs.erase(contentName) > 0){
        sigExtraSeqsChanged_();
    }
}


namespace cnoid {

BodyMotion::Frame& operator<<(const BodyMotion::Frame& frame, const Body& body)
{
    BodyMotion& motion = const_cast<BodyMotion::Frame&>(frame).motion();
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
    return const_cast<BodyMotion::Frame&>(frame);
}

BodyMotion::Frame& operator>>(const BodyMotion::Frame& frame, Body& body)
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
    /*
    if(numLinks <= 1){
        body.calcForwardKinematics();
    }
    */
    return const_cast<BodyMotion::Frame&>(frame);
}
}

/*
  bool BodyMotion::setBodyState(int frameIndex, const Body& body)
  {
  bool processed = false;

  const int n = std::min(body->numJoints(), jointPosSeq_->numParts());
  if(n > 0 && frameIndex < jointPosSeq_->numFrames()){
  MultiValueSeq::Frame jframe = jointPosSeq_->frame(frameIndex);
  for(int i=0; i < n; ++i){
  jframe[i] = body->joint(i)->q;
  }
  processed = true;
  }
  const int n = std::min(body->numLinks(), linkPosSeq_->numParts());
  if(n > 0 && frameIndex < linkPosSeq_->numFrames()){
  MultiSE3Seq::Frame lframe = linkPosSeq_->frame(frameIndex);
  for(int i=0; i < n; ++i){
  Link* link = body->link(n);
  lframe[i].set(link->p, link->R);
  }
  processed = true;
  }
  return processed;
  }


  bool BodyMotion::getBodyState(int frameIndex, Body& body)
  {
  bool processed = false;
    
  const int n = std::min(body->numJoints(), jointPosSeq_->numParts());
  if(n > 0 && frameIndex < jointPosSeq_->numFrames()){
  const MultiValueSeq::Frame jframe = jointPosSeq_->frame(frameIndex);
  for(int i=0; i < n; ++i){
  body->joint(i)->q = jframe[i];
  }
  processed = true;
  }
  const int n = std::min(body->numLinks(), linkPosSeq_->numParts());
  if(n > 0 && frameIndex < linkPosSeq_->numFrames()){
  const MultiSE3Seq::Frame lframe = linkPosSeq_->frame(frameIndex);
  for(int i=0; i < n; ++i){
  Link* link = body->link(n);
  const SE3& pos = lframe[i];
  link->p = pos.translation();
  link->R = pos.rotation();
  }
  processed = true;
  }
  return processed;
  }
*/
