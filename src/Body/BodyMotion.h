/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_BODY_BODY_MOTION_H
#define CNOID_BODY_BODY_MOTION_H

#include <cnoid/MultiValueSeq>
#include <cnoid/MultiSE3Seq>
#include <cnoid/Signal>
#include <boost/make_shared.hpp>
#include <map>
#include "exportdecl.h"

namespace cnoid {

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

    virtual int getOffsetTimeFrame() const;

    int numFrames() const {
        return std::max(jointPosSeq_->numFrames(), linkPosSeq_->numFrames());
    }
    virtual int getNumFrames() const;
    virtual void setNumFrames(int n, bool clearNewArea = false);

    MultiValueSeqPtr& jointPosSeq() {
        return jointPosSeq_;
    }

    const MultiValueSeqPtr& jointPosSeq() const {
        return jointPosSeq_;
    }

    MultiSE3SeqPtr& linkPosSeq() {
        return linkPosSeq_;
    }

    const MultiSE3SeqPtr& linkPosSeq() const {
        return linkPosSeq_;
    }

    class Frame {
        const BodyMotion& motion_;
        int frame_;
        Frame(const BodyMotion& motion, int frame) : motion_(motion), frame_(frame) { }
    public:
        Frame(const Frame& org) : motion_(org.motion_), frame_(org.frame_) { }
        inline BodyMotion& motion() { return const_cast<BodyMotion&>(motion_); }
        inline const BodyMotion& motion() const { return motion_; }
        inline int frame() const { return frame_; }

        friend class BodyMotion;
    };
            
    Frame frame(int frame) { return Frame(*this, frame); }
    const Frame frame(int frame) const { return Frame(*this, frame); }

    virtual bool read(const Mapping& archive);
    virtual bool write(YAMLWriter& writer);

    bool loadStandardYAMLformat(const std::string& filename);
    bool saveAsStandardYAMLformat(const std::string& filename);

    typedef std::map<std::string, AbstractSeqPtr> ExtraSeqMap;
    typedef ExtraSeqMap::const_iterator ConstSeqIterator;
        
    ConstSeqIterator extraSeqBegin() const { return extraSeqs.begin(); }
    ConstSeqIterator extraSeqEnd() const { return extraSeqs.end(); }
        
    template <class SeqType>
        boost::shared_ptr<SeqType> extraSeq(const std::string& contentName) const {
        ExtraSeqMap::const_iterator p = extraSeqs.find(contentName);
        return ((p != extraSeqs.end()) ?
                boost::dynamic_pointer_cast<SeqType>(p->second) : boost::shared_ptr<SeqType>());
    }

    void setExtraSeq(AbstractSeqPtr seq);

    template <class SeqType>
        boost::shared_ptr<SeqType> getOrCreateExtraSeq(const std::string& contentName) {
        AbstractSeqPtr& base = extraSeqs[contentName];
        boost::shared_ptr<SeqType> seq;
        if(base){
            seq = boost::dynamic_pointer_cast<SeqType>(base);
        }
        if(!seq){
            seq = boost::make_shared<SeqType>(numFrames());
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

typedef boost::shared_ptr<BodyMotion> BodyMotionPtr;

class Body;

CNOID_EXPORT BodyMotion::Frame& operator<<(const BodyMotion::Frame& frame, const Body& body);
CNOID_EXPORT BodyMotion::Frame& operator>>(const BodyMotion::Frame& frame, const Body& body);
}

#endif
