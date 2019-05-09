/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_ZMP_SEQ_H
#define CNOID_BODY_ZMP_SEQ_H

#include <cnoid/Vector3Seq>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ZMPSeq : public Vector3Seq
{
public:
    typedef Vector3Seq BaseSeqType;
    
    static const std::string& key();

    ZMPSeq(int nFrames = 0);
    ZMPSeq(const ZMPSeq& org);
    ZMPSeq(const Vector3Seq& org);

    virtual AbstractSeq& operator=(const AbstractSeq& rhs) override;
    ZMPSeq& operator=(const ZMPSeq& rhs);
    virtual std::shared_ptr<AbstractSeq> cloneSeq() const override;

    bool isRootRelative() const { return isRootRelative_; }
    void setRootRelative(bool on);

protected:
    virtual bool doReadSeq(const Mapping* archive, std::ostream& os) override;
    virtual bool doWriteSeq(YAMLWriter& writer, std::function<void()> additionalPartCallback) override;

private:
    bool isRootRelative_;
};

class BodyMotion;

CNOID_EXPORT std::shared_ptr<ZMPSeq> getZMPSeq(const BodyMotion& motion);
CNOID_EXPORT std::shared_ptr<ZMPSeq> getOrCreateZMPSeq(BodyMotion& motion);
CNOID_EXPORT void clearZMPSeq(BodyMotion& motion);
CNOID_EXPORT bool makeRootRelative(ZMPSeq& zmpseq, BodyMotion& motion, bool on);

}

#endif
