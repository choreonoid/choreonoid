/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MULTI_SEQ_H
#define CNOID_UTIL_MULTI_SEQ_H

#include "AbstractSeq.h"
#include "Deque2D.h"
#include <algorithm>
#include <memory>

namespace cnoid {

template <typename ElementType, typename Allocator = std::allocator<ElementType>>
class MultiSeq : public Deque2D<ElementType, Allocator>, public AbstractMultiSeq
{
    typedef MultiSeq<ElementType, Allocator> MultiSeqType;
        
public:
    typedef Deque2D<ElementType, Allocator> Container;
    
    typedef typename Container::value_type value_type;
    typedef typename Container::Row Frame;
    typedef typename Container::Column Part;
    typedef value_type Element; ///< \deprecated. Use value_type.

    MultiSeq(const char* seqType)
        : Container(0, 1),
          AbstractMultiSeq(seqType) {
        frameRate_ = 0.0;
        offsetTime_ = 0.0;
    }

    MultiSeq(const char* seqType, int numFrames, int numParts)
        : Container(numFrames, numParts),
          AbstractMultiSeq(seqType) {
        frameRate_ = 0.0;
        offsetTime_ = 0.0;
    }

    MultiSeq(const MultiSeqType& org)
        : Container(org),
          AbstractMultiSeq(org) {
        frameRate_ = org.frameRate_;
        offsetTime_ = org.offsetTime_;
    }
    
    virtual ~MultiSeq() { }

    MultiSeqType& operator=(const MultiSeqType& rhs){
        if(this != &rhs){
            AbstractMultiSeq::operator=(rhs);
            Container::operator=(rhs);
            frameRate_ = rhs.frameRate_;
            offsetTime_ = rhs.offsetTime_;
        }
        return *this;
    }

    virtual AbstractSeq& operator=(const AbstractSeq& rhs) override {
        const MultiSeqType* rhsSeq = dynamic_cast<const MultiSeqType*>(&rhs);
        if(rhsSeq){
            return operator=(*rhsSeq);
        } else {
            return AbstractSeq::operator=(rhs);
        }
    }

    virtual std::shared_ptr<AbstractSeq> cloneSeq() const override {
        return std::make_shared<MultiSeqType>(*this);
    }
        
    void copySeqProperties(const MultiSeqType& source) {
        AbstractMultiSeq::copySeqProperties(source);
        frameRate_ = source.frameRate_;
        setNumParts(source.numParts());
    }
            
    virtual void setDimension(int newNumFrames, int newNumParts, bool fillNewElements = false) override {

        const int prevNumParts = numParts();
        const int prevNumFrames = numFrames();

        Container::resize(newNumFrames, newNumParts);

        if(fillNewElements){
            if(newNumParts != prevNumParts){
                std::fill(Container::begin(), Container::end(), defaultValue());
            } else {
                if(newNumFrames > prevNumFrames){
                    if(prevNumFrames == 0){
                        std::fill(Container::begin() + prevNumFrames * newNumParts, Container::end(), defaultValue());
                    } else {
                        Frame last = frame(prevNumFrames - 1);
                        for(int i=prevNumFrames; i < newNumFrames; ++i){
                            std::copy(last.begin(), last.end(), frame(i).begin());
                        }
                    }
                }
            }
        }
    }

    virtual double getFrameRate() const override {
        return frameRate_;
    }

    double frameRate() const {
        return frameRate_;
    }

    virtual void setFrameRate(double frameRate) override {
        frameRate_ = frameRate;
    }

    const double timeStep() const {
        const double r = getFrameRate();
        return (r > 0.0) ? 1.0 / r : 0.0;
    }

    virtual void setNumParts(int newNumParts, bool fillNewElements = false) override {
        setDimension(numFrames(), newNumParts, fillNewElements);
    }

    int numFrames() const {
        return Container::rowSize();
    }

    virtual int getNumFrames() const override {
        return numFrames();
    }

    virtual void setNumFrames(int newNumFrames, bool fillNewElements = false) override {
        setDimension(newNumFrames, numParts(), fillNewElements);
    }

    void clearFrames(){
        setNumFrames(0);
    }

    virtual int getNumParts() const override {
        return Container::colSize();
    }

    int numParts() const {
        return Container::colSize();
    }

    double timeLength() const {
        return (frameRate_ > 0.0) ? (numFrames() / frameRate_) : 0.0;
    }

    int frameOfTime(double time) const {
        return static_cast<int>((time - offsetTime_) * frameRate_);
    }
            
    double timeOfFrame(int frame) const {
        return (frameRate_ > 0.0) ? ((frame / frameRate_) + offsetTime_) : offsetTime_;
    }

    double offsetTime() const {
        return offsetTime_;
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

    virtual int getOffsetTimeFrame() const override {
        return static_cast<int>(offsetTime_ * frameRate_);
    }

    virtual void setOffsetTimeFrame(int offset) override {
        offsetTime_ = (frameRate_ > 0) ? (offset / frameRate_) : 0.0;
    }
    
    const Part part(int index) const {
        return Container::column(index);
    }

    Part part(int index) {
        return Container::column(index);
    }

    Frame frame(int index) {
        return Container::row(index);
    }

    const Frame frame(int index) const {
        return Container::row(index);
    }

    void popFrontFrame() {
        Container::pop_front();
    }

    Frame appendFrame() {
        return Container::append();
    }

    int clampFrameIndex(int frameIndex){
        if(frameIndex < 0){
            return 0;
        } else if(frameIndex >= numFrames()){
            return numFrames() - 1;
        }
        return frameIndex;
    }

protected:
    double frameRate_;
    double offsetTime_;

    virtual value_type defaultValue() const { return value_type(); }
};

}

#endif
