/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SEQ_H
#define CNOID_UTIL_SEQ_H

#include "AbstractSeq.h"
#include <memory>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

template <typename ElementType> class Seq : public AbstractSeq
{
    typedef Seq<ElementType> SeqType;
        
public:
    typedef ElementType value_type;
        
    Seq(const char* seqType, int nFrames = 0.0)
        : AbstractSeq(seqType),
          container(nFrames)
    {
        frameRate_ = defaultFrameRate();
        offsetTime_ = 0.0;
    }
        
    Seq(const SeqType& org)
        : AbstractSeq(org),
          container(org.container)
    {
        frameRate_ = org.frameRate_;
        offsetTime_ = org.offsetTime_;
    }

    SeqType& operator=(const SeqType& rhs)
    {
        if(this != &rhs){
            AbstractSeq::operator=(rhs);
            container = rhs.container;
            frameRate_ = rhs.frameRate_;
            offsetTime_ = rhs.offsetTime_;
        }
        return *this;
    }

    virtual AbstractSeq& operator=(const AbstractSeq& rhs) override {
        const SeqType* rhsSeq = dynamic_cast<const SeqType*>(&rhs);
        if(rhsSeq){
            return operator=(*rhsSeq);
        } else {
            return AbstractSeq::operator=(rhs);
        }
    }

    virtual std::shared_ptr<AbstractSeq> cloneSeq() const override {
        return std::make_shared<SeqType>(*this);
    }
        
    virtual ~Seq() { }
        
    double frameRate() const {
        return frameRate_;
    }

    virtual double getFrameRate() const override {
        return frameRate_;
    }

    virtual void setFrameRate(double frameRate) override {
        frameRate_ = frameRate;
    }

    int numFrames() const {
        return container.size();
    }

    virtual int getNumFrames() const override {
        return container.size();
    }

    virtual void setNumFrames(int n, bool fillNewElements = false) override {
        if(fillNewElements){
            const int nold = numFrames();
            if(n > nold && nold > 0){
                container.resize(n, container[nold - 1]);
            } else {
                container.resize(n, defaultValue());
            }
        } else {
            container.resize(n);
        }
    }

    void clear() {
        container.clear();
    }

    bool empty() const {
        return container.empty();
    }

    double timeLength() const {
        return (frameRate_ > 0.0) ? (numFrames() / frameRate_) : 0.0;
    }

    int frameOfTime(double time) const {
        return static_cast<int>((time - offsetTime_) * frameRate_);
    }

    int lastFrameOfTime(double time) const {
        int frame = frameOfTime(time);
        const int n = numFrames();
        if(frame >= n){
            frame = (n > 0) ? (n - 1) : 0;
        }
        return frame;
    }
    
    double timeOfFrame(int frame) const {
        return (frameRate_ > 0.0) ? ((frame / frameRate_) + offsetTime_) : offsetTime_;
    }

    virtual double getOffsetTime() const override {
        return offsetTime_;
    }

    virtual void setOffsetTime(double time) override {
        offsetTime_ = time;
    }

    int offsetTimeFrame() const {
        return static_cast<int>(offsetTime_ * frameRate_);
    }

    virtual void setOffsetTimeFrame(int offset) override {
        offsetTime_ = (frameRate_ > 0) ? (offset / frameRate_) : 0.0;
    }
    
    ElementType& operator[](int frameIndex) {
        return container[frameIndex];
    }

    const ElementType& operator[](int frameIndex) const {
        return container[frameIndex];
    }

    ElementType& at(int frameIndex) {
        return container[frameIndex];
    }

    const ElementType& at(int frameIndex) const {
        return container[frameIndex];
    }

    ElementType& front() {
        return container.front();
    }

    const ElementType& front() const {
        return container.front();
    }

    ElementType& back() {
        return container.back();
    }

    const ElementType& back() const {
        return container.back();
    }

    int clampFrameIndex(int frameIndex, bool& out_isValidRange){
        if(frameIndex < 0){
            frameIndex = 0;
            out_isValidRange = false;
        } else if(frameIndex >= numFrames()){
            frameIndex = numFrames() - 1;
            out_isValidRange = false;
        } else {
            out_isValidRange = true;
        }
        return frameIndex;
    }

protected:
    std::vector<ElementType> container;
    double frameRate_;
    double offsetTime_;

    virtual ElementType defaultValue() const { return ElementType(); }
};

}

#endif
