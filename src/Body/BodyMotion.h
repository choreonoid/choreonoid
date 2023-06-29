#ifndef CNOID_BODY_BODY_MOTION_H
#define CNOID_BODY_BODY_MOTION_H

#include <cnoid/BodyPositionSeq>
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
    virtual std::shared_ptr<AbstractSeq> cloneSeq() const override;

    double frameRate() const { return positionSeq_->frameRate(); }
    virtual double getFrameRate() const override;
    virtual void setFrameRate(double frameRate) override;

    double timeStep() const { return positionSeq_->timeStep(); }

    double offsetTime() const { return positionSeq_->offsetTime(); }
    virtual double getOffsetTime() const override;
    virtual void setOffsetTime(double time) override;

    int numFrames() const { return positionSeq_->numFrames(); }
    virtual int getNumFrames() const override;
    virtual void setNumFrames(int n, bool fillNewElements = false) override;

    std::shared_ptr<BodyPositionSeq> positionSeq() { return positionSeq_; }
    std::shared_ptr<const BodyPositionSeq> positionSeq() const { return positionSeq_; }

    void setDimension(int numFrames, int numJoints, int numLinks, bool fillNewElements = false);
    void setNumJoints(int numJoints, bool fillNewElements = false);
    int numLinks() const { return positionSeq_->numLinkPositionsHint(); }
    int numJoints() const { return positionSeq_->numJointDisplacementsHint(); }

    //[[deprecated("Use positionSeq.")]]
    std::shared_ptr<MultiSE3Seq> linkPosSeq();
    //[[deprecated("Use positionSeq.")]]
    std::shared_ptr<const MultiSE3Seq> linkPosSeq() const;
    //[[deprecated("Use positionSeq.")]]
    std::shared_ptr<MultiValueSeq> jointPosSeq();
    //[[deprecated("Use positionSeq.")]]
    std::shared_ptr<const MultiValueSeq> jointPosSeq() const;

    /*
      The linkPosSeq and jointPosSeq data members were replaced with a new data format,
      the positionSeq member. The codes that use the old members should be modified to
      use the new member, or you can convert the data by inserting the following functions.
    */
    void updateLinkPosSeqWithBodyPositionSeq();
    void updateJointPosSeqWithBodyPositionSeq();
    void updateLinkPosSeqAndJointPosSeqWithBodyPositionSeq();
    void updateBodyPositionSeqWithLinkPosSeqAndJointPosSeq();

    static const std::string& linkPositionContentName();
    [[deprecated("Use linkPosSeqContentName")]]
    static const std::string& linkPosSeqKey() { return linkPositionContentName(); }
    static const std::string& jointDisplacementContentName();
    [[deprecated("Use jointPosSeqContentName")]]
    static const std::string& jointPosSeqKey() { return jointDisplacementContentName(); }

    static const std::string& jointEffortContentName();

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

    bool hasExtraSeqs() const { return !extraSeqs.empty(); }
    ConstSeqIterator extraSeqBegin() const { return extraSeqs.begin(); }
    ConstSeqIterator extraSeqEnd() const { return extraSeqs.end(); }
        
    template <class SeqType>
    std::shared_ptr<SeqType> extraSeq(const std::string& contentName) const {
        ExtraSeqMap::const_iterator p = extraSeqs.find(contentName);
        return ((p != extraSeqs.end()) ?
                std::dynamic_pointer_cast<SeqType>(p->second) : std::shared_ptr<SeqType>());
    }

    void setExtraSeq(std::shared_ptr<AbstractSeq> seq);

    template <class SeqType>
    std::shared_ptr<SeqType> getOrCreateExtraSeq(
        const std::string& contentName, std::function<void(SeqType& seq)> initFunc = nullptr) {
        std::shared_ptr<AbstractSeq>& base = extraSeqs[contentName];
        std::shared_ptr<SeqType> seq;
        if(base){
            seq = std::dynamic_pointer_cast<SeqType>(base);
        }
        if(!seq){
            seq = std::make_shared<SeqType>();
            base = seq;
            seq->setSeqContentName(contentName);
            seq->setFrameRate(frameRate());
            seq->setNumFrames(numFrames());
            seq->setOffsetTime(offsetTime());
            if(initFunc){
                initFunc(*seq);
            }
            sigExtraSeqsChanged_();
        }
        return seq;
    }

    void clearExtraSeqs();
    void clearExtraSeq(const std::string& contentName);

    SignalProxy<void()> sigExtraSeqsChanged() {
        return sigExtraSeqsChanged_;
    }

    [[deprecated("Use setNumJoints.")]]
    void setNumParts(int numJoints, bool fillNewElements = false){
        setNumJoints(numJoints, fillNewElements);
    }
    [[deprecated("Use numJoints.")]]
    int getNumParts() const { return numJoints(); }

    [[deprecated("Use load.")]]
    bool loadStandardYAMLformat(const std::string& filename, std::ostream& os = nullout()){
        return load(filename, os);
    }
    [[deprecated("Use save.")]]
    bool saveAsStandardYAMLformat(const std::string& filename, std::ostream& os = nullout()){
        return save(filename, os);
    }

protected:
    virtual bool doReadSeq(const Mapping* archive, std::ostream& os) override;
    virtual bool doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback) override;
        
private:
    std::shared_ptr<MultiSE3Seq> getOrCreateLinkPosSeq();
    std::shared_ptr<MultiValueSeq> getOrCreateJointPosSeq();
    
    std::shared_ptr<BodyPositionSeq> positionSeq_;
    ExtraSeqMap extraSeqs;
    Signal<void()> sigExtraSeqsChanged_;
};

CNOID_EXPORT BodyMotion::Frame operator<<(BodyMotion::Frame frame, const Body& body);
CNOID_EXPORT BodyMotion::Frame operator>>(BodyMotion::Frame frame, Body& body);
CNOID_EXPORT BodyMotion::ConstFrame operator>>(BodyMotion::ConstFrame frame, Body& body);

CNOID_EXPORT Body& operator<<(Body& body, BodyMotion::Frame frame);
CNOID_EXPORT Body& operator<<(Body& body, BodyMotion::ConstFrame frame);
CNOID_EXPORT const Body& operator>>(const Body& body, BodyMotion::Frame frame);

}

#endif
