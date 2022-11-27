#ifndef CNOID_BODY_BODY_POSITION_SEQ_H
#define CNOID_BODY_BODY_POSITION_SEQ_H

#include <cnoid/Seq>
#include <cnoid/EigenTypes>
#include <vector>
#include <algorithm>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT BodyPositionSeqFrameBlock
{
public:
    static constexpr int LinkPositionSize = 7;

    BodyPositionSeqFrameBlock(double* pdata) : pdata(pdata) { }

    bool empty() const { return pdata == nullptr; }
    explicit operator bool() const { return !empty(); }

    int numLinkPositions() const {
        return pdata ? pdata[0] : 0;
    }

    int numJointDisplacements() const {
        return pdata ? pdata[static_cast<int>(pdata[0]) * LinkPositionSize + 1] : 0;
    }

    // The order of the link position elements is x, y, z, qx, qy, qz, qw
    class LinkPosition {
    public:
        LinkPosition(double* pdata) : pdata(pdata) { }

        LinkPosition& operator=(const LinkPosition& rhs){
            std::copy(rhs.pdata, rhs.pdata + 7, pdata);
            return *this;
        }

        Eigen::Map<Vector3> translation() { return Eigen::Map<Vector3>(pdata); }
        Eigen::Map<const Vector3> translation() const { return Eigen::Map<const Vector3>(pdata); }
        Eigen::Map<Quaternion> rotation() { return Eigen::Map<Quaternion>(pdata + 3); }
        Eigen::Map<const Quaternion> rotation() const { return Eigen::Map<const Quaternion>(pdata + 3); }
        const Isometry3 position() const {
            Isometry3 T;
            T.translation() = translation();
            T.linear() = rotation().toRotationMatrix();
            return T;
        }
        const Isometry3 T() const {
            return position();
        }

        void set(const Isometry3& T){
            Eigen::Map<Vector3> translation(pdata);
            translation = T.translation();
            Eigen::Map<Quaternion> rotation(pdata + 3);
            rotation = T.linear();
        }
        void set(const SE3& position){
            Eigen::Map<Vector3> translation(pdata);
            translation = position.translation();
            Eigen::Map<Quaternion> rotation(pdata + 3);
            rotation = position.rotation();
        }
        
    private:
        double* pdata;
    };

    LinkPosition linkPosition(int index) {
        return LinkPosition(pdata + index * LinkPositionSize + 1);
    }

    const LinkPosition linkPosition(int index) const {
        return LinkPosition(const_cast<double*>(pdata + index * LinkPositionSize + 1));
    }

    double* linkPositionData(){
        return pdata ? pdata + 1 : nullptr;
    }

    const double* linkPositionData() const {
        return pdata ? pdata + 1 : nullptr;
    }
    
    double& jointDisplacement(int index){
        return pdata[static_cast<int>(pdata[0]) * LinkPositionSize + index + 2];
    }

    double jointDisplacement(int index) const {
        return const_cast<BodyPositionSeqFrameBlock*>(this)->jointDisplacement(index);
    }

    double* jointDisplacements(){
        return pdata + static_cast<int>(pdata[0]) * LinkPositionSize + 2;
    }

    const double* jointDisplacements() const {
        return const_cast<BodyPositionSeqFrameBlock*>(this)->jointDisplacements();
    }

protected:
    BodyPositionSeqFrameBlock() : pdata(nullptr) { }
    
    double* pdata;

    friend class BodyPositionSeqFrame;
};


class CNOID_EXPORT BodyPositionSeqFrame : public BodyPositionSeqFrameBlock
{
public:
    BodyPositionSeqFrame();
    BodyPositionSeqFrame(const BodyPositionSeqFrame& org);
    BodyPositionSeqFrame(BodyPositionSeqFrame&& org);
    
    BodyPositionSeqFrame& operator=(const BodyPositionSeqFrame& rhs){
        data = rhs.data;
        pdata = data.data();
        return *this;
    }

    BodyPositionSeqFrame& operator=(BodyPositionSeqFrame&& rhs){
        data = std::move(rhs.data);
        pdata = data.data();
        return *this;
    }

    void clear(){
        data.clear();
        pdata = nullptr;
    }
    
    BodyPositionSeqFrame& allocate(int numLinks, int numJoints){
        data.resize(numLinks * LinkPositionSize + numJoints + 2);
        data[0] = numLinks;
        data[numLinks * LinkPositionSize + 1] = numJoints;
        pdata = data.data();
        return *this;
    }

    BodyPositionSeqFrameBlock extend(int numLinks, int numJoints){
        int prevSize = data.size();
        data.resize(prevSize + numLinks * LinkPositionSize + numJoints + 2);
        pdata = data.data();
        double* block = pdata + prevSize;
        block[0] = numLinks;
        block[numLinks * LinkPositionSize + 1] = numJoints;
        return BodyPositionSeqFrameBlock(block);
    }

    BodyPositionSeqFrameBlock firstBlock(){
        return BodyPositionSeqFrameBlock(data.empty() ? nullptr : data.data());
    }

    const BodyPositionSeqFrameBlock firstBlock() const {
        return const_cast<BodyPositionSeqFrame*>(this)->firstBlock();
    }
    
    BodyPositionSeqFrameBlock nextBlockOf(const BodyPositionSeqFrameBlock& block){
        double* nextData = block.pdata + block.numLinkPositions() * LinkPositionSize + block.numJointDisplacements() + 2;
        if(nextData >= data.data() + data.size()){
            nextData = nullptr;
        }
        return BodyPositionSeqFrameBlock(nextData);
    }

    const BodyPositionSeqFrameBlock nextBlockOf(const BodyPositionSeqFrameBlock& block) const {
        return const_cast<BodyPositionSeqFrame*>(this)->nextBlockOf(block);
    }
    
private:
    std::vector<double> data;
};


/**
   \note This class can contain multiple body states
   \note 
   Frame elements:
   nlinks, x, y, z, qx, qy, qz, qw, .... , njoints, q0, q1, q2, ..., nlinks, x, y, z, ...
*/
class CNOID_EXPORT BodyPositionSeq : public Seq<BodyPositionSeqFrame>
{
public:
    BodyPositionSeq(int numFrames = 0);
    BodyPositionSeq(const BodyPositionSeq& org);

    int numLinkPositionsHint() const { return numLinkPositionsHint_; }
    void setNumLinkPositionsHint(int n) { numLinkPositionsHint_ = n; }
    
    int numJointDisplacementsHint() const { return numJointDisplacementsHint_; }
    void setNumJointDisplacementsHint(int n) { numJointDisplacementsHint_ = n; }

    BodyPositionSeqFrame& appendAllocatedFrame(){
        return append().allocate(numLinkPositionsHint_, numJointDisplacementsHint_);
    }

    BodyPositionSeqFrame& allocateFrame(int index, int numLinks, int numJoints){
        if(index >= numFrames()){
            setNumFrames(index + 1);
        }
        return frame(index).allocate(numLinks, numJoints);
    }

    BodyPositionSeqFrame& allocateFrame(int index){
        return allocateFrame(index, numLinkPositionsHint_, numJointDisplacementsHint_);
    }

    // TODO: Implement the following functions
    // LinkPositionSeq linkPositionSeq(int linkIndex);
    // JointDisplacementSeq jointDisplacementSeq(int jointId);

private:
    int numLinkPositionsHint_;
    int numJointDisplacementsHint_;
};

class Body;

CNOID_EXPORT BodyPositionSeqFrame& operator<<(BodyPositionSeqFrame& frame, const Body& body);
CNOID_EXPORT BodyPositionSeqFrame& operator>>(BodyPositionSeqFrame& frame, Body& body);
CNOID_EXPORT const BodyPositionSeqFrame& operator>>(const BodyPositionSeqFrame& frame, Body& body);
CNOID_EXPORT Body& operator<<(Body& body, const BodyPositionSeqFrame& frame);
CNOID_EXPORT Body& operator>>(Body& body, BodyPositionSeqFrame& frame);
CNOID_EXPORT const Body& operator>>(const Body& body, BodyPositionSeqFrame& frame);

}

#endif
