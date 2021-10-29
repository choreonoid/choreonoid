/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_ABSTRACT_SEQ_H
#define CNOID_UTIL_ABSTRACT_SEQ_H

#include "NullOut.h"
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class Mapping;
class YAMLWriter;
    
class CNOID_EXPORT AbstractSeq
{
protected:
    AbstractSeq(const char* seqType);
    AbstractSeq(const AbstractSeq& org);
    
public:
    virtual ~AbstractSeq();

    virtual AbstractSeq& operator=(const AbstractSeq& rhs);

    virtual std::shared_ptr<AbstractSeq> cloneSeq() const = 0;

    void copySeqProperties(const AbstractSeq& source);

    const std::string& seqType() const {
        return seqType_;
    }

    virtual double getFrameRate() const = 0;
    virtual void setFrameRate(double frameRate) = 0;
    static double defaultFrameRate();
        
    double getTimeStep() const;
    void setTimeStep(double timeStep);

    double getTimeOfFrame(int frame) const;
    int getFrameOfTime(double time) const;

    virtual double getOffsetTime() const = 0;
    virtual void setOffsetTime(double time) = 0;
    virtual int getOffsetTimeFrame() const;
    virtual void setOffsetTimeFrame(int offset);
    
    virtual int getNumFrames() const = 0;
    virtual void setNumFrames(int n, bool clearNewElements = false) = 0;

    void setTimeLength(double length, bool clearNewElements = false){
        return setNumFrames(static_cast<int>(length * getFrameRate()), clearNewElements);
    }

    /**
       This function returns the duration of the sequence.
       @note Valid data exists for time less than this time.
       You must not access to the frame corresponding to this time because
       there is no data at the time.
    */
    double getTimeLength() const;

    const std::string& seqContentName() {
        return contentName_;
    }

    virtual void setSeqContentName(const std::string& name);

    bool readSeq(const Mapping* archive, std::ostream& os = nullout());
    bool writeSeq(YAMLWriter& writer);

    //! deprecated. Use the os parameter of readSeq to get messages in reading
    const std::string& seqMessage() const;

protected:
    void setSeqType(const std::string& type);
    
    virtual bool doReadSeq(const Mapping* archive, std::ostream& os);
    virtual bool doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback);

    //! deprecated. Use the os parameter of readSeq to get messages in reading
    void clearSeqMessage() { }

    //! deprecated. Use the os parameter of readSeq to get messages in reading
    void addSeqMessage(const std::string& message) { }

private:
    std::string seqType_;
    std::string contentName_;
};


class CNOID_EXPORT AbstractMultiSeq : public AbstractSeq
{
public:
    AbstractMultiSeq(const char* seqType);
    AbstractMultiSeq(const AbstractMultiSeq& org);
    virtual ~AbstractMultiSeq();

    using AbstractSeq::operator=;
    AbstractMultiSeq& operator=(const AbstractMultiSeq& rhs);

    virtual std::shared_ptr<AbstractSeq> cloneSeq() const = 0;

    void copySeqProperties(const AbstractMultiSeq& source);

    virtual void setDimension(int numFrames, int numParts, bool clearNewElements = false) = 0;
            
    virtual void setNumParts(int numParts, bool clearNewElements = false) = 0;
    virtual int getNumParts() const = 0;

    virtual int partIndex(const std::string& partLabel) const;
    virtual const std::string& partLabel(int partIndex) const;

protected:
    virtual bool doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback);
    std::vector<std::string> readSeqPartLabels(const Mapping* archive);
    bool writeSeqPartLabels(YAMLWriter& writer);
};

#ifdef CNOID_BACKWARD_COMPATIBILITY
typedef std::shared_ptr<AbstractSeq> AbstractSeqPtr;
typedef std::shared_ptr<AbstractMultiSeq> AbstractMultiSeqPtr;
#endif

}

#endif
