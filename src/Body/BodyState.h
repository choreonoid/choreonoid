#ifndef CNOID_BODY_BODY_STATE_H
#define CNOID_BODY_BODY_STATE_H

#include "Device.h"
#include <cnoid/EigenTypes>
#include <vector>
#include <algorithm>
#include "exportdecl.h"

namespace cnoid {

class Body;

class CNOID_EXPORT BodyStateBlock
{
public:
    static constexpr int HeaderSize = 4;
    static constexpr int LinkPositionSize = 7;

    bool empty() const { return pData == nullptr; }
    explicit operator bool() const { return !empty(); }

    int numLinkPositions() const {
        return pData ? pData[0] : 0;
    }

    bool hasLinkPositions() const {
        return pData && (pData[0] > 0);
    }

    int numJointDisplacements() const {
        return pData ? pData[1] : 0;
    }

    bool hasJointDisplacements() const {
        return pData && (pData[1] > 0);
    }

    int numDeviceStates() const {
        return pData ? pData[2] : 0;
    }

    bool hasDeviceStates() const {
        return pData && (pData[2] > 0);
    }

    // The order of the link position elements is x, y, z, qx, qy, qz, qw
    class LinkPosition {
    public:
        LinkPosition(double* pData) : pData(pData) { }

        LinkPosition& operator=(const LinkPosition& rhs){
            std::copy(rhs.pData, rhs.pData + LinkPositionSize, pData);
            return *this;
        }

        Eigen::Map<Vector3> translation() { return Eigen::Map<Vector3>(pData); }
        Eigen::Map<const Vector3> translation() const { return Eigen::Map<const Vector3>(pData); }
        Eigen::Map<Quaternion> rotation() { return Eigen::Map<Quaternion>(pData + 3); }
        Eigen::Map<const Quaternion> rotation() const { return Eigen::Map<const Quaternion>(pData + 3); }
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
            Eigen::Map<Vector3> translation(pData);
            translation = T.translation();
            Eigen::Map<Quaternion> rotation(pData + 3);
            rotation = T.linear();
        }
        void set(const SE3& position){
            Eigen::Map<Vector3> translation(pData);
            translation = position.translation();
            Eigen::Map<Quaternion> rotation(pData + 3);
            rotation = position.rotation();
        }
        void get(Isometry3& out_T) const {
            Eigen::Map<Vector3> translation(pData);
            out_T.translation() = translation;
            Eigen::Map<Quaternion> rotation(pData + 3);
            out_T.linear() = rotation.toRotationMatrix();
        }
        void get(SE3& out_position) const {
            Eigen::Map<Vector3> translation(pData);
            Eigen::Map<Quaternion> rotation(pData + 3);
            out_position.translation() = translation;
            out_position.rotation() = rotation;
        }

    private:
        double* pData;
    };

    LinkPosition linkPosition(int index) {
        return LinkPosition(pData + HeaderSize + index * LinkPositionSize);
    }

    const LinkPosition linkPosition(int index) const {
        return LinkPosition(const_cast<double*>(pData + HeaderSize + index * LinkPositionSize));
    }

    LinkPosition rootLinkPosition() {
        return linkPosition(0);
    }

    const LinkPosition rootLinkPosition() const {
        return linkPosition(0);
    }

    double* linkPositionData(){
        return pData ? pData + HeaderSize : nullptr;
    }

    const double* linkPositionData() const {
        return pData ? pData + HeaderSize : nullptr;
    }

    double& jointDisplacement(int index){
        return pData[HeaderSize + static_cast<int>(pData[0]) * LinkPositionSize + index];
    }

    double jointDisplacement(int index) const {
        return const_cast<BodyStateBlock*>(this)->jointDisplacement(index);
    }

    double* jointDisplacements(){
        return pData + HeaderSize + static_cast<int>(pData[0]) * LinkPositionSize;
    }

    const double* jointDisplacements() const {
        return const_cast<BodyStateBlock*>(this)->jointDisplacements();
    }

    DeviceState* deviceState(int index) {
        return pDeviceData[index];
    }

    const DeviceState* deviceState(int index) const {
        return pDeviceData[index];
    }

    void setDeviceState(int index, DeviceState* state) {
        pDeviceData[index] = state;
    }

    void storeStateOfBody(const Body* body);
    bool restoreStateToBody(Body* body) const;

protected:
    BodyStateBlock() { }
    BodyStateBlock(double* pData, DeviceStatePtr* pDeviceData) : pData(pData), pDeviceData(pDeviceData) { }

    double* pData;
    DeviceStatePtr* pDeviceData;

    friend class BodyState;
};


/**
   \note This class can contain multiplex body states

   \note
   Elements:
   n links, n joints, n device states, device state offset, x, y, z, qx, qy, qz, qw, .... , q0, q1, q2, ...
*/
class CNOID_EXPORT BodyState : public BodyStateBlock
{
public:
    BodyState() : BodyStateBlock(nullptr, nullptr) { }

    BodyState(const BodyState& org)
        : data(org.data), deviceData(org.deviceData) {
        pData = data.data();
        pDeviceData = deviceData.data();
    }

    BodyState(BodyState&& org)
        : data(std::move(org.data)), deviceData(std::move(org.deviceData)) {
        pData = data.data();
        pDeviceData = deviceData.data();
    }

    BodyState(const Body* body);

    [[deprecated("Use BodyState(const Body* body)")]]
    BodyState(const Body& body);

    BodyState& operator=(const BodyState& rhs){
        data = rhs.data;
        pData = data.data();
        deviceData = rhs.deviceData;
        pDeviceData = deviceData.data();
        return *this;
    }

    BodyState& operator=(BodyState&& rhs){
        data = std::move(rhs.data);
        pData = data.data();
        deviceData = std::move(rhs.deviceData);
        pDeviceData = deviceData.data();
        return *this;
    }

    void clear(){
        data.clear();
        pData = nullptr;
        deviceData.clear();
        pDeviceData = nullptr;
    }

    BodyState& allocate(int numLinks, int numJoints = 0, int numDevices = 0){
        data.resize(HeaderSize + numLinks * LinkPositionSize + numJoints);
        data[0] = numLinks;
        data[1] = numJoints;
        data[2] = numDevices;
        if(numDevices > 0){
            deviceData.resize(numDevices);
            data[3] = 0; // offset in device data
        }
        pData = data.data();
        pDeviceData = deviceData.data();
        return *this;
    }

    BodyStateBlock extend(int numLinks, int numJoints = 0, int numDevices = 0){
        int prevDataSize = data.size();
        data.resize(prevDataSize + HeaderSize + numLinks * LinkPositionSize + numJoints);
        pData = data.data();
        double* block = pData + prevDataSize;
        block[0] = numLinks;
        block[1] = numJoints;
        block[2] = numDevices;
        if(numDevices > 0){
            block[3] = deviceData.size();
            deviceData.resize(deviceData.size() + numDevices);
        }
        pDeviceData = deviceData.data();
        return BodyStateBlock(block, pDeviceData);
    }

    BodyStateBlock firstBlock(){
        return BodyStateBlock(data.empty() ? nullptr : data.data(), deviceData.data());
    }

    const BodyStateBlock firstBlock() const {
        return const_cast<BodyState*>(this)->firstBlock();
    }
    
    BodyStateBlock nextBlockOf(const BodyStateBlock& block){
        double* nextData = block.pData + HeaderSize + block.numLinkPositions() * LinkPositionSize + block.numJointDisplacements();
        if(nextData >= data.data() + data.size()){
            nextData = nullptr;
            return BodyStateBlock(nextData, nullptr);
        }
        return BodyStateBlock(nextData, deviceData.data() + static_cast<int>(nextData[3]));
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
    std::vector<DeviceStatePtr> deviceData;
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
