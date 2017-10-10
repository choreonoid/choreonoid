/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_BODY_BODY_MOTION_H
#define CNOID_BODY_BODY_MOTION_H

#include <cnoid/MultiValueSeq>
#include <cnoid/MultiSE3Seq>
#include <cnoid/Signal>
#include <map>
#include "exportdecl.h"

namespace cnoid {

class Body;

class CNOID_EXPORT BodyMotion : public AbstractMultiSeq
{
public:
    BodyMotion();
    BodyMotion(const BodyMotion& org);
            
    BodyMotion& operator=(const BodyMotion& rhs);
    virtual AbstractSeqPtr cloneSeq() const;        

    virtual void setDimension(int numFrames, int numJoints, bool clearNewArea = false);

    void setDimension(int numFrames, int numJoints, int numLinks, bool clearNewArea = false);

    virtual void setNumParts(int numParts, bool clearNewElements = false);
    virtual int getNumParts() const;

    int numJoints() const { return jointPosSeq_->numParts(); }
    int numLinks() const { return linkPosSeq_->numParts(); }

    double frameRate() const { return jointPosSeq_->frameRate(); }
    virtual double getFrameRate() const;
    virtual void setFrameRate(double frameRate);

    double timeStep() const { return jointPosSeq_->timeStep(); }

    virtual int getOffsetTimeFrame() const;

    int numFrames() const {
        return std::max(jointPosSeq_->numFrames(), linkPosSeq_->numFrames());
    }
    virtual int getNumFrames() const;
    virtual void setNumFrames(int n, bool clearNewArea = false);

    MultiValueSeqPtr jointPosSeq() {
        return jointPosSeq_;
    }

    ConstMultiValueSeqPtr jointPosSeq() const {
        return jointPosSeq_;
    }

    MultiSE3SeqPtr linkPosSeq() {
        return linkPosSeq_;
    }

    ConstMultiSE3SeqPtr linkPosSeq() const {
        return linkPosSeq_;
    }

    class CNOID_EXPORT Frame {
        BodyMotion& motion_;
        int frame_;
        Frame(BodyMotion& motion, int frame) : motion_(motion), frame_(frame) { }
        Frame& operator=(const Frame& rhs) { return *this; }
    public:
        Frame();
        Frame(const Frame& org) : motion_(org.motion_), frame_(org.frame_) { }
        ~Frame() { }
        BodyMotion& motion() { return motion_; }
        int frame() const { return frame_; }
        friend class BodyMotion;
    };

    class ConstFrame {
        const BodyMotion& motion_;
        int frame_;
        ConstFrame(const BodyMotion& motion, int frame) : motion_(motion), frame_(frame) { }
        ConstFrame& operator=(const ConstFrame& rhs) { return *this; }
        ConstFrame& operator=(const Frame& rhs) { return *this; }
    public:
        ConstFrame();
        ConstFrame(const ConstFrame& org) : motion_(org.motion_), frame_(org.frame_) { }
        ConstFrame(const Frame& org) : motion_(org.motion_), frame_(org.frame_) { }
        ~ConstFrame() { }
        const BodyMotion& motion() const { return motion_; }
        int frame() const { return frame_; }
        friend class BodyMotion;
    };
    
    Frame frame(int frame) { return Frame(*this, frame); }
    ConstFrame frame(int frame) const { return ConstFrame(*this, frame); }

    virtual bool read(const Mapping& archive);
    virtual bool write(YAMLWriter& writer);

    bool loadStandardYAMLformat(const std::string& filename);
    bool saveAsStandardYAMLformat(const std::string& filename);

    typedef std::map<std::string, AbstractSeqPtr> ExtraSeqMap;
    typedef ExtraSeqMap::const_iterator ConstSeqIterator;
        
    ConstSeqIterator extraSeqBegin() const { return extraSeqs.begin(); }
    ConstSeqIterator extraSeqEnd() const { return extraSeqs.end(); }
        
    template <class SeqType>
        std::shared_ptr<SeqType> extraSeq(const std::string& contentName) const {
        ExtraSeqMap::const_iterator p = extraSeqs.find(contentName);
        return ((p != extraSeqs.end()) ?
                std::dynamic_pointer_cast<SeqType>(p->second) : std::shared_ptr<SeqType>());
    }

    void setExtraSeq(AbstractSeqPtr seq);

    template <class SeqType>
        std::shared_ptr<SeqType> getOrCreateExtraSeq(const std::string& contentName) {
        AbstractSeqPtr& base = extraSeqs[contentName];
        std::shared_ptr<SeqType> seq;
        if(base){
            seq = std::dynamic_pointer_cast<SeqType>(base);
        }
        if(!seq){
            seq = std::make_shared<SeqType>(numFrames());
            seq->setFrameRate(frameRate());
            base = seq;
            sigExtraSeqsChanged_();
        }
        return seq;
    }

    void clearExtraSeq(const std::string& contentName);

    SignalProxy<void()> sigExtraSeqsChanged() {
        return sigExtraSeqsChanged_;
    }
        
private:

    MultiValueSeqPtr jointPosSeq_;
    MultiSE3SeqPtr linkPosSeq_;

    ExtraSeqMap extraSeqs;

    Signal<void()> sigExtraSeqsChanged_;
};

typedef std::shared_ptr<BodyMotion> BodyMotionPtr;
typedef std::shared_ptr<const BodyMotion> ConstBodyMotionPtr;

class Body;

CNOID_EXPORT BodyMotion::Frame operator<<(BodyMotion::Frame frame, const Body& body);
CNOID_EXPORT BodyMotion::Frame operator>>(BodyMotion::Frame frame, const Body& body);
CNOID_EXPORT BodyMotion::ConstFrame operator>>(BodyMotion::ConstFrame frame, Body& body);
CNOID_EXPORT Body& operator<<(Body& body, BodyMotion::Frame frame);
CNOID_EXPORT Body& operator<<(Body& body, BodyMotion::ConstFrame frame);
CNOID_EXPORT const Body& operator>>(const Body& body, BodyMotion::Frame frame);

}

#endif
