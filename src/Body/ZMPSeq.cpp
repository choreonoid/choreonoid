/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#include "ZMPSeq.h"
#include "BodyMotion.h"

using namespace std;
using namespace cnoid;

namespace {
static const string zmpkey("ZMP");
}
    

ZMPSeq::ZMPSeq(int nFrames)
    : Vector3Seq(nFrames)
{
    setSeqContentName(zmpkey);
    isRootRelative_ = false;
}


ZMPSeq::ZMPSeq(const ZMPSeq& org)
    : Vector3Seq(org)
{
    isRootRelative_ = org.isRootRelative_;
}


ZMPSeq::ZMPSeq(const Vector3Seq& org)
    : Vector3Seq(org)
{
    setSeqContentName(zmpkey);
    isRootRelative_ = false;
}


ZMPSeq& ZMPSeq::operator=(const ZMPSeq& rhs)
{
    if(this != &rhs){
        Vector3Seq::operator=(rhs);
        isRootRelative_ = rhs.isRootRelative_;
    }
    return *this;
}


AbstractSeq& ZMPSeq::operator=(const AbstractSeq& rhs)
{
    const ZMPSeq* rhsZmpSeq = dynamic_cast<const ZMPSeq*>(&rhs);
    if(rhsZmpSeq){
        return operator=(*rhsZmpSeq);
    } else {
        return Vector3Seq::BaseSeqType::operator=(rhs);
    }
}


AbstractSeqPtr ZMPSeq::cloneSeq() const
{
    return boost::make_shared<ZMPSeq>(*this);
}


const std::string& ZMPSeq::key()
{
    return zmpkey;
}


ZMPSeqPtr cnoid::getZMPSeq(const BodyMotion& motion)
{
    return motion.extraSeq<ZMPSeq>(zmpkey);
}


ZMPSeqPtr cnoid::getOrCreateZMPSeq(BodyMotion& motion)
{
    return motion.getOrCreateExtraSeq<ZMPSeq>(zmpkey);
}


void cnoid::clearZMPSeq(BodyMotion& motion)
{
    motion.clearExtraSeq(zmpkey);
}


bool cnoid::makeRootRelative(ZMPSeq& zmpseq, BodyMotion& motion, bool on)
{
    if(zmpseq.isRootRelative() != on){
        MultiSE3Seq::Part rootSeq = motion.linkPosSeq()->part(0);
        if(rootSeq.empty()){
            return false;
        }
        if(on){
            // make the coordinate root relative
            for(int i=0; i < zmpseq.numFrames(); ++i){
                const SE3& p = rootSeq[std::min(i, rootSeq.size() - 1)];
                zmpseq[i] = p.rotation().inverse() * zmpseq[i] - p.translation();
            }
        } else {
            // make the coordinate global
            for(int i=0; i < zmpseq.numFrames(); ++i){
                const SE3& p = rootSeq[std::min(i, rootSeq.size() - 1)];
                zmpseq[i] = p.rotation() * zmpseq[i] + p.translation();
            }
        }
        zmpseq.setRootRelative(on);
    }
    return true;
}

