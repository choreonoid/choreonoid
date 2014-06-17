/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MULTI_SEQ_H_INCLUDED
#define CNOID_UTIL_MULTI_SEQ_H_INCLUDED

#include <Eigen/StdVector>
#include "AbstractSeq.h"
#include "Deque2D.h"
#include <boost/make_shared.hpp>
#include <algorithm>

namespace cnoid {

template <typename ElementType, typename Allocator = std::allocator<ElementType> >
class MultiSeq : public Deque2D<ElementType, Allocator>, public AbstractMultiSeq
{
    typedef MultiSeq<ElementType, Allocator> MultiSeqType;
        
public:
    typedef Deque2D<ElementType, Allocator> Deque2DType;
    typedef typename Deque2DType::Element Element;
    typedef boost::shared_ptr< MultiSeqType > Ptr;
    typedef typename Deque2DType::Row Frame;
    typedef typename Deque2DType::Column Part;

    MultiSeq(const char* seqType)
        : AbstractMultiSeq(seqType),
          Deque2DType(0, 1) {
        frameRate_ = defaultFrameRate();
    }

    MultiSeq(const char* seqType, int numFrames, int numParts)
        : AbstractMultiSeq(seqType),
          Deque2DType(numFrames, numParts) {
        frameRate_ = defaultFrameRate();
    }

    MultiSeq(const MultiSeqType& org)
        : AbstractMultiSeq(org),
          Deque2DType(org) {
        frameRate_ = org.frameRate_;
    }
    
    virtual ~MultiSeq() { }

    MultiSeqType& operator=(const MultiSeqType& rhs) {
        if(this != &rhs){
            AbstractMultiSeq::operator=(rhs);
            Deque2DType::operator=(rhs);
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

        Deque2DType::resize(newNumFrames, newNumParts);

        if(clearNewElements){
            if(newNumParts == prevNumParts){
                if(newNumFrames > prevNumFrames){
                    std::fill(Deque2DType::begin() + prevNumFrames * newNumParts,
                              Deque2DType::end(),
                              defaultValue());
                }
            } else {
                std::fill(Deque2DType::begin(), Deque2DType::end(), defaultValue());
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
        return Deque2DType::rowSize();
    }

    inline int numFrames() const {
        return Deque2DType::rowSize();
    }

    virtual void setNumFrames(int newNumFrames, bool clearNewElements = false) {
        setDimension(newNumFrames, numParts(), clearNewElements);
    }

    virtual int getNumParts() const {
        return Deque2DType::colSize();
    }

    inline int numParts() const {
        return Deque2DType::colSize();
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
        return Deque2DType::column(index);
    }

    inline Part part(int index) {
        return Deque2DType::column(index);
    }

    inline Frame frame(int index) {
        return Deque2DType::row(index);
    }

    inline const Frame frame(int index) const {
        return Deque2DType::row(index);
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
