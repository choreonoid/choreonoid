/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_BODY_BODY_MOTION_H
#define CNOID_BODY_BODY_MOTION_H

#include <cnoid/MultiValueSeq>
#include <cnoid/MultiSE3Seq>
#include <cnoid/Signal>
#include <cnoid/NullOut>
#include <map>
#include "exportdecl.h"

namespace cnoid {

class Body;

class CNOID_EXPORT BodyMotion : public AbstractSeq
{
public:
    BodyMotion();
    BodyMotion(const BodyMotion& org);

    using AbstractSeq::operator=;
    BodyMotion& operator=(const BodyMotion& rhs);
    virtual std::shared_ptr<AbstractSeq> cloneSeq() const;        

    void setDimension(int numFrames, int numJoints, int numLinks, bool clearNewArea = false);
    void setNumJoints(int numJoints, bool clearNewElements = false);

    int numLinks() const { return linkPosSeq_->numParts(); }
    int numJoints() const { return jointPosSeq_->numParts(); }

    double frameRate() const;
    virtual double getFrameRate() const override;
    virtual void setFrameRate(double frameRate) override;

    double timeStep() const;

    virtual double getOffsetTime() const override;
    virtual void setOffsetTime(double time) override;

    int numFrames() const;
    virtual int getNumFrames() const override;
    virtual void setNumFrames(int n, bool clearNewArea = false) override;

    std::shared_ptr<MultiSE3Seq> linkPosSeq() {
        return linkPosSeq_;
    }

    std::shared_ptr<const MultiSE3Seq> linkPosSeq() const {
        return linkPosSeq_;
    }

    std::shared_ptr<MultiValueSeq> jointPosSeq() {
        return jointPosSeq_;
    }

    std::shared_ptr<const MultiValueSeq> jointPosSeq() const {
        return jointPosSeq_;
    }

    class CNOID_EXPORT Frame {
        BodyMotion& motion_;
        int frame_;
        Frame(BodyMotion& motion, int frame) : motion_(motion), frame_(frame) { }
        Frame& operator=(const Frame&) { return *this; }
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
        ConstFrame& operator=(const ConstFrame&) { return *this; }
        ConstFrame& operator=(const Frame&) { return *this; }
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

    bool load(const std::string& filename, std::ostream& os = nullout());
    bool save(const std::string& filename, std::ostream& os = nullout());
    bool save(const std::string& filename, double version, std::ostream& os = nullout());

    typedef std::map<std::string, std::shared_ptr<AbstractSeq>> ExtraSeqMap;
    typedef ExtraSeqMap::const_iterator ConstSeqIterator;
        
    ConstSeqIterator extraSeqBegin() const { return extraSeqs.begin(); }
    ConstSeqIterator extraSeqEnd() const { return extraSeqs.end(); }
        
    template <class SeqType>
        std::shared_ptr<SeqType> extraSeq(const std::string& name) const {
        ExtraSeqMap::const_iterator p = extraSeqs.find(name);
        return ((p != extraSeqs.end()) ?
                std::dynamic_pointer_cast<SeqType>(p->second) : std::shared_ptr<SeqType>());
    }

    void setExtraSeq(const std::string& name, std::shared_ptr<AbstractSeq> seq);

    template <class SeqType>
        std::shared_ptr<SeqType> getOrCreateExtraSeq(const std::string& name) {
        std::shared_ptr<AbstractSeq>& base = extraSeqs[name];
        std::shared_ptr<SeqType> seq;
        if(base){
            seq = std::dynamic_pointer_cast<SeqType>(base);
        }
        if(!seq){
            seq = std::make_shared<SeqType>();
            base = seq;
            seq->setFrameRate(frameRate());
            seq->setNumFrames(numFrames());
            sigExtraSeqsChanged_();
        }
        return seq;
    }

    void clearExtraSeq(const std::string& name);

    SignalProxy<void()> sigExtraSeqsChanged() {
        return sigExtraSeqsChanged_;
    }

    //! \deprecated
    void setNumParts(int numJoints, bool clearNewElements = false){
        setNumJoints(numJoints, clearNewElements);
    }

    //! \deprecated
    int getNumParts() const { return numJoints(); }
    
    //! \deprecated
    bool loadStandardYAMLformat(const std::string& filename, std::ostream& os = nullout()){
        return load(filename, os);
    }
    //! \deprecated
    bool saveAsStandardYAMLformat(const std::string& filename, std::ostream& os = nullout()){
        return save(filename, os);
    }

protected:
    virtual bool doReadSeq(const Mapping* archive, std::ostream& os) override;
    virtual bool doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback) override;
        
private:
    std::shared_ptr<MultiSE3Seq> linkPosSeq_;
    std::shared_ptr<MultiValueSeq> jointPosSeq_;
    ExtraSeqMap extraSeqs;
    Signal<void()> sigExtraSeqsChanged_;
};

CNOID_EXPORT BodyMotion::Frame operator<<(BodyMotion::Frame frame, const Body& body);
CNOID_EXPORT BodyMotion::Frame operator>>(BodyMotion::Frame frame, const Body& body);
CNOID_EXPORT BodyMotion::ConstFrame operator>>(BodyMotion::ConstFrame frame, Body& body);
CNOID_EXPORT Body& operator<<(Body& body, BodyMotion::Frame frame);
CNOID_EXPORT Body& operator<<(Body& body, BodyMotion::ConstFrame frame);
CNOID_EXPORT const Body& operator>>(const Body& body, BodyMotion::Frame frame);

#ifdef CNOID_BACKWARD_COMPATIBILITY
typedef std::shared_ptr<BodyMotion> BodyMotionPtr;
#endif

}

#endif
