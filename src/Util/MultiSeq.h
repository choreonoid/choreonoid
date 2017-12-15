/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MULTI_SEQ_H
#define CNOID_UTIL_MULTI_SEQ_H

#include "AbstractSeq.h"
#include "Deque2D.h"
#include <Eigen/StdVector>
#include <algorithm>
#include <memory>


namespace cnoid {

template <typename ElementType, typename Allocator = std::allocator<ElementType> >
class MultiSeq : public Deque2D<ElementType, Allocator>, public AbstractMultiSeq
{
    typedef MultiSeq<ElementType, Allocator> MultiSeqType;
        
public:
    typedef Deque2D<ElementType, Allocator> Container;
    
    typedef typename Container::Element Element;
    typedef std::shared_ptr< MultiSeqType > Ptr;
    typedef typename Container::Row Frame;
    typedef typename Container::Column Part;

    MultiSeq(const char* seqType)
        : Container(0, 1),
          AbstractMultiSeq(seqType) {
        frameRate_ = defaultFrameRate();
        offsetTimeFrame_ = 0;
    }

    MultiSeq(const char* seqType, int numFrames, int numParts)
        : Container(numFrames, numParts),
          AbstractMultiSeq(seqType) {
        frameRate_ = defaultFrameRate();
        offsetTimeFrame_ = 0;
    }

    MultiSeq(const MultiSeqType& org)
        : Container(org),
          AbstractMultiSeq(org) {
        frameRate_ = org.frameRate_;
        offsetTimeFrame_ = org.offsetTimeFrame_;
    }
    
    virtual ~MultiSeq() { }

    MultiSeqType& operator=(const MultiSeqType& rhs){
        if(this != &rhs){
            AbstractMultiSeq::operator=(rhs);
            Container::operator=(rhs);
            frameRate_ = rhs.frameRate_;
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

    virtual AbstractSeqPtr cloneSeq() const override {
        return std::make_shared<MultiSeqType>(*this);
    }
        
    void copySeqProperties(const MultiSeqType& source) {
        AbstractMultiSeq::copySeqProperties(source);
        frameRate_ = source.frameRate_;
        setNumParts(source.numParts());
    }
            
    virtual void setDimension(int newNumFrames, int newNumParts, bool clearNewElements = false) override {

        const int prevNumParts = numParts();
        const int prevNumFrames = numFrames();

        Container::resize(newNumFrames, newNumParts);

        if(clearNewElements){
            if(newNumParts == prevNumParts){
                if(newNumFrames > prevNumFrames){
                    std::fill(Container::begin() + prevNumFrames * newNumParts,
                              Container::end(),
                              defaultValue());
                }
            } else {
                std::fill(Container::begin(), Container::end(), defaultValue());
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
        return 1.0 / frameRate_;
    }

    virtual void setNumParts(int newNumParts, bool clearNewElements = false) override {
        setDimension(numFrames(), newNumParts, clearNewElements);
    }

    virtual int getNumFrames() const override {
        return Container::rowSize();
    }

    int numFrames() const {
        return Container::rowSize();
    }

    virtual void setNumFrames(int newNumFrames, bool clearNewElements = false) override {
        setDimension(newNumFrames, numParts(), clearNewElements);
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
        return numFrames() / frameRate();
    }

    virtual bool setOffsetTimeFrame(int frameOffset) override {
        offsetTimeFrame_ = frameOffset;
        return true;
    }

    int offsetTimeFrame() const {
        return offsetTimeFrame_;
    }

    virtual int getOffsetTimeFrame() const override {
        return offsetTimeFrame_;
    }

    int frameOfTime(double time) const {
        return (int)(time * frameRate_) - offsetTimeFrame_;
    }
            
    double timeOfFrame(int frame) const {
        return ((frame + offsetTimeFrame_) / frameRate_);
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
        offsetTimeFrame_ += 1;
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
    int offsetTimeFrame_;

    virtual ElementType defaultValue() const { return ElementType(); }
};

}

#endif
