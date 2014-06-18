/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MULTI_SEQ_H
#define CNOID_UTIL_MULTI_SEQ_H

#include <Eigen/StdVector>
#include "AbstractSeq.h"
#include <boost/make_shared.hpp>
#include <algorithm>

#define USE_DEQUE_FOR_MULTI_SEQ 1

#if USE_DEQUE_FOR_MULTI_SEQ
#include "Deque2D.h"
#else
#include "Array2D.h"
#endif


namespace cnoid {

template <typename ElementType, typename Allocator = std::allocator<ElementType> >
#if USE_DEQUE_FOR_MULTI_SEQ
class MultiSeq : public Deque2D<ElementType, Allocator>, public AbstractMultiSeq
#else
class MultiSeq : public Array2D<ElementType, Allocator>, public AbstractMultiSeq
#endif
{
    typedef MultiSeq<ElementType, Allocator> MultiSeqType;
        
public:
#if USE_DEQUE_FOR_MULTI_SEQ
    typedef Deque2D<ElementType, Allocator> Container;
#else
    typedef Array2D<ElementType, Allocator> Container;
#endif
    
    typedef typename Container::Element Element;
    typedef boost::shared_ptr< MultiSeqType > Ptr;
    typedef typename Container::Row Frame;
    typedef typename Container::Column Part;

    MultiSeq(const char* seqType)
        : AbstractMultiSeq(seqType),
          Container(0, 1) {
        frameRate_ = defaultFrameRate();
    }

    MultiSeq(const char* seqType, int numFrames, int numParts)
        : AbstractMultiSeq(seqType),
          Container(numFrames, numParts) {
        frameRate_ = defaultFrameRate();
    }

    MultiSeq(const MultiSeqType& org)
        : AbstractMultiSeq(org),
          Container(org) {
        frameRate_ = org.frameRate_;
    }
    
    virtual ~MultiSeq() { }

    MultiSeqType& operator=(const MultiSeqType& rhs) {
        if(this != &rhs){
            AbstractMultiSeq::operator=(rhs);
            Container::operator=(rhs);
            frameRate_ = rhs.frameRate_;
        }
        return *this;
    }

    virtual AbstractSeq& operator=(const AbstractSeq& rhs) {
        const MultiSeqType* rhsSeq = dynamic_cast<const MultiSeqType*>(&rhs);
        if(rhsSeq){
            return operator=(*rhsSeq);
        } else {
            return AbstractSeq::operator=(rhs);
        }
    }

    virtual AbstractSeqPtr cloneSeq() const {
        return boost::make_shared<MultiSeqType>(*this);
    }
        
    void copySeqProperties(const MultiSeqType& source) {
        AbstractMultiSeq::copySeqProperties(source);
        frameRate_ = source.frameRate_;
        setNumParts(source.numParts());
    }
            
    virtual void setDimension(int newNumFrames, int newNumParts, bool clearNewElements = false) {

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

    virtual double getFrameRate() const {
        return frameRate_;
    }

    inline double frameRate() const {
        return frameRate_;
    }

    virtual void setFrameRate(double frameRate) {
        frameRate_ = frameRate;
    }

    const double timeStep() const {
        return 1.0 / frameRate_;
    }

    virtual void setNumParts(int newNumParts, bool clearNewElements = false) {
        setDimension(numFrames(), newNumParts, clearNewElements);
    }

    virtual int getNumFrames() const {
        return Container::rowSize();
    }

    inline int numFrames() const {
        return Container::rowSize();
    }

    virtual void setNumFrames(int newNumFrames, bool clearNewElements = false) {
        setDimension(newNumFrames, numParts(), clearNewElements);
    }

    virtual int getNumParts() const {
        return Container::colSize();
    }

    inline int numParts() const {
        return Container::colSize();
    }

    inline double timeLength() const {
        return numFrames() / frameRate();
    }

    inline int frameOfTime(double time) const {
        return (int)(time * frameRate_);
    }
            
    inline double timeOfFrame(int frame) const {
        return (frame / frameRate_);
    }

    inline const Part part(int index) const {
        return Container::column(index);
    }

    inline Part part(int index) {
        return Container::column(index);
    }

    inline Frame frame(int index) {
        return Container::row(index);
    }

    inline const Frame frame(int index) const {
        return Container::row(index);
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

    double frameRate_;

    virtual ElementType defaultValue() const { return ElementType(); }
};

}

#endif
