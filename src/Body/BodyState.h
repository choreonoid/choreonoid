#ifndef CNOID_BODY_BODY_STATE_H
#define CNOID_BODY_BODY_STATE_H

#include <cnoid/EigenTypes>
#include <vector>
#include <algorithm>
#include "exportdecl.h"

namespace cnoid {

class Body;

class CNOID_EXPORT BodyStateBlock
{
public:
    static constexpr int LinkPositionSize = 7;

    BodyStateBlock(double* pdata) : pdata(pdata) { }

    bool empty() const { return pdata == nullptr; }
    explicit operator bool() const { return !empty(); }

    int numLinkPositions() const {
        return pdata ? pdata[0] : 0;
    }

    bool hasLinkPositions() const {
        return pdata && (pdata[0] > 0);
    }

    int numJointDisplacements() const {
        return pdata ? pdata[static_cast<int>(pdata[0]) * LinkPositionSize + 1] : 0;
    }

    bool hasJointDisplacements() const {
        return pdata && (pdata[static_cast<int>(pdata[0]) * LinkPositionSize + 1] > 0);
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
        void get(Isometry3& out_T) const {
            Eigen::Map<Vector3> translation(pdata);
            out_T.translation() = translation;
            Eigen::Map<Quaternion> rotation(pdata + 3);
            out_T.linear() = rotation.toRotationMatrix();
        }
        void get(SE3& out_position) const {
            Eigen::Map<Vector3> translation(pdata);
            Eigen::Map<Quaternion> rotation(pdata + 3);
            out_position.translation() = translation;
            out_position.rotation() = rotation;
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

    LinkPosition rootLinkPosition() {
        return linkPosition(0);
    }

    const LinkPosition rootLinkPosition() const {
        return linkPosition(0);
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
        return const_cast<BodyStateBlock*>(this)->jointDisplacement(index);
    }

    double* jointDisplacements(){
        return pdata + static_cast<int>(pdata[0]) * LinkPositionSize + 2;
    }

    const double* jointDisplacements() const {
        return const_cast<BodyStateBlock*>(this)->jointDisplacements();
    }

    void storeStateOfBody(const Body* body);
    bool restoreStateToBody(Body* body) const;

protected:
    BodyStateBlock() : pdata(nullptr) { }
    
    double* pdata;

    friend class BodyState;
};


/**
   \note This class can contain multiplex body states

   \note
   Elements:
   nlinks, x, y, z, qx, qy, qz, qw, .... , njoints, q0, q1, q2, ..., nlinks, x, y, z, ...
*/
class CNOID_EXPORT BodyState : public BodyStateBlock
{
public:
    BodyState() { }
    BodyState(const BodyState& org) : data(org.data) { pdata = data.data(); }
    BodyState(BodyState&& org) : data(std::move(org.data)) { pdata = data.data(); }

    BodyState(const Body* body);

    [[deprecated("Use BodyState(const Body* body)")]]
    BodyState(const Body& body);

    BodyState& operator=(const BodyState& rhs){
        data = rhs.data;
        pdata = data.data();
        return *this;
    }

    BodyState& operator=(BodyState&& rhs){
        data = std::move(rhs.data);
        pdata = data.data();
        return *this;
    }

    void clear(){
        data.clear();
        pdata = nullptr;
    }

    BodyState& allocate(int numLinks, int numJoints){
        data.resize(numLinks * LinkPositionSize + numJoints + 2);
        data[0] = numLinks;
        data[numLinks * LinkPositionSize + 1] = numJoints;
        pdata = data.data();
        return *this;
    }

    BodyStateBlock extend(int numLinks, int numJoints){
        int prevSize = data.size();
        data.resize(prevSize + numLinks * LinkPositionSize + numJoints + 2);
        pdata = data.data();
        double* block = pdata + prevSize;
        block[0] = numLinks;
        block[numLinks * LinkPositionSize + 1] = numJoints;
        return BodyStateBlock(block);
    }

    BodyStateBlock firstBlock(){
        return BodyStateBlock(data.empty() ? nullptr : data.data());
    }

    const BodyStateBlock firstBlock() const {
        return const_cast<BodyState*>(this)->firstBlock();
    }
    
    BodyStateBlock nextBlockOf(const BodyStateBlock& block){
        double* nextData = block.pdata + block.numLinkPositions() * LinkPositionSize + block.numJointDisplacements() + 2;
        if(nextData >= data.data() + data.size()){
            nextData = nullptr;
        }
        return BodyStateBlock(nextData);
    }

    const BodyStateBlock nextBlockOf(const BodyStateBlock& block) const {
        return const_cast<BodyState*>(this)->nextBlockOf(block);
    }

    void storeStateOfBody(const Body* body);

    [[deprecated("Use storeStateOfBody.")]]
    void storePositions(const Body& body){
        storeStateOfBody(&body);
    }

    [[deprecated("Use restoreStateToBody.")]]
    bool restorePositions(Body& body) const {
        return restoreStateToBody(&body);
    }

private:
    std::vector<double> data;
};

CNOID_EXPORT BodyState& operator<<(BodyState& state, const Body& body);
CNOID_EXPORT BodyState& operator>>(BodyState& state, Body& body);
CNOID_EXPORT const BodyState& operator>>(const BodyState& state, Body& body);

CNOID_EXPORT Body& operator<<(Body& body, const BodyState& state);
CNOID_EXPORT Body& operator>>(Body& body, BodyState& state);
CNOID_EXPORT const Body& operator>>(const Body& body, BodyState& state);

[[deprecated("Use BodyStateBlock.")]]
typedef BodyStateBlock BodyPositionSeqFrameBlock;

[[deprecated("Use BodyState.")]]
typedef BodyState BodyPositionSeqFrame;

}

#endif
