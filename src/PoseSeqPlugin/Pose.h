/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_POSE_SEQ_PLUGIN_POSE_H
#define CNOID_POSE_SEQ_PLUGIN_POSE_H

#include <cnoid/Body>
#include <map>
#include "exportdecl.h"

namespace cnoid {

class Mapping;
    
class PoseUnit;
class PoseSeq;
class PoseRef;

typedef ref_ptr<PoseUnit> PoseUnitPtr;

class CNOID_EXPORT PoseUnit : public Referenced
{
public:
    PoseUnit();
    PoseUnit(const PoseUnit& org);
    virtual ~PoseUnit();

    virtual PoseUnit* duplicate() = 0;

    virtual bool restore(const Mapping& archive, const BodyPtr body) = 0;
    virtual void store(Mapping& archive, const BodyPtr body) const = 0;
    virtual bool hasSameParts(PoseUnitPtr unit) { return false; }

    /**
       @note A name can be only set by PoseSeq::rename().
    */
    const std::string& name() const {
        return name_;
    }
            
private:
    std::string name_;
    PoseSeq* owner;
    int seqLocalReferenceCounter;

    friend class PoseSeq;
    friend class PoseRef;
};

        
class CNOID_EXPORT Pose : public PoseUnit
{
    struct JointInfo {
        JointInfo() : isValid(false), isStationaryPoint(false) { }
        double q;
        bool isValid;
        bool isStationaryPoint;
    };
            
public:

    class LinkInfo {

    public:

        Vector3 p;
        Matrix3 R;

        LinkInfo() :
            isBaseLink_(false),
            isStationaryPoint_(false),
            isTouching_(false),
            isSlave_(false) { }
        bool isBaseLink() const { return isBaseLink_; }
        void setStationaryPoint(bool on){ isStationaryPoint_ = on; }
        bool isStationaryPoint() const { return isStationaryPoint_; }
        bool isTouching() const { return isTouching_; }
        const Vector3& partingDirection() const { return partingDirection_; }
        const std::vector<Vector3>& contactPoints() const { return contactPoints_; }
        void setTouching(const Vector3& partingDirection, const std::vector<Vector3>& contactPoints) {
            isTouching_ = true;
            partingDirection_ = partingDirection;
            contactPoints_ = contactPoints;
        }
        void clearTouching() { isTouching_ = false; }
        bool isSlave() const { return isSlave_; }
        void setSlave(bool on) { isSlave_ = on; }
                
    private:
        bool isBaseLink_;
        bool isStationaryPoint_;
        bool isTouching_;
        bool isSlave_;
        Vector3 partingDirection_;
        std::vector<Vector3> contactPoints_;
        friend class Pose;
    };

    typedef std::map<int, LinkInfo> LinkInfoMap;

    Pose();
    Pose(int numJoints);
    Pose(const Pose& org);
            
    virtual ~Pose();

    bool empty();

    void clear();

    virtual PoseUnit* duplicate();

    virtual bool hasSameParts(PoseUnitPtr unit);

    virtual bool restore(const Mapping& archive, const BodyPtr body);
    virtual void store(Mapping& archive, const BodyPtr body) const;

    void setNumJoints(int n){
        jointInfos.resize(n);
    }
            
    int numJoints() const {
        return jointInfos.size();
    }

    void setJointPosition(int jointId, double q){
        if(jointId >= 0){
            if(jointId >= (int)jointInfos.size()){
                setNumJoints(jointId + 1);
            }
            JointInfo& info = jointInfos[jointId];
            info.q = q;
            info.isValid = true;
        }
    }

    double jointPosition(int jointId) const {
        return jointInfos[jointId].q;
    }

    bool isJointValid(int jointId) const {
        if(jointId < 0 || jointId >= (int)jointInfos.size()){
            return false;
        }
        return jointInfos[jointId].isValid;
    }

    void setJointStationaryPoint(int jointId, bool on = true){
        if(jointId >= (int)jointInfos.size()){
            setNumJoints(jointId + 1);
        }
        jointInfos[jointId].isStationaryPoint = on;
    }

    bool isJointStationaryPoint(int jointId) const {
        if(jointId >= (int)jointInfos.size()){
            return false;
        }
        return jointInfos[jointId].isStationaryPoint;
    }

    bool invalidateJoint(int jointId) {
        if(jointId < (int)jointInfos.size()){
            if(jointInfos[jointId].isValid){
                jointInfos[jointId].isValid = false;
                return true;
            }
        }
        return false;
    }

    void clearIkLinks();

    size_t numIkLinks(){
        return ikLinks.size();
    }

    LinkInfo* addIkLink(int linkIndex){
        return &ikLinks[linkIndex];
    }

    bool removeIkLink(int linkIndex);

    const LinkInfo* ikLinkInfo(int linkIndex) const {
        LinkInfoMap::const_iterator p = ikLinks.find(linkIndex);
        return (p != ikLinks.end()) ? &p->second : 0;
    }

    LinkInfo* ikLinkInfo(int linkIndex) {
        LinkInfoMap::iterator p = ikLinks.find(linkIndex);
        return (p != ikLinks.end()) ? &p->second : 0;
    }
            
    LinkInfoMap::iterator ikLinkBegin() { return ikLinks.begin(); }
    const LinkInfoMap::const_iterator ikLinkBegin() const { return ikLinks.begin(); }
    LinkInfoMap::iterator ikLinkEnd() { return ikLinks.end(); }
    const LinkInfoMap::const_iterator ikLinkEnd() const { return ikLinks.end(); }

    LinkInfo& setBaseLink(int linkIndex);

    LinkInfo& setBaseLink(int linkIndex, const Vector3& p, const Matrix3& R){
        LinkInfo& info = setBaseLink(linkIndex);
        info.p = p;
        info.R = R;
        return info;
    }
            
    int baseLinkIndex() const {
        return (baseLinkIter != ikLinks.end()) ? baseLinkIter->first : -1;
    }
            
    LinkInfo* baseLinkInfo() {
        return (baseLinkIter != ikLinks.end()) ? &baseLinkIter->second : 0;
    }

    void invalidateBaseLink() {
        if(baseLinkIter != ikLinks.end()){
            baseLinkIter->second.isBaseLink_ = false;
            baseLinkIter = ikLinks.end();
        }
    }

    void setZmp(const Vector3& p){
        isZmpValid_ = true;
        zmp_ = p;
    }
            
    const Vector3 zmp() const {
        return zmp_;
    }
            
    bool isZmpValid() const {
        return isZmpValid_;
    }
            
    bool invalidateZmp() {
        bool ret = isZmpValid_;
        isZmpValid_ = false;
        return ret;
    }
            
    void setZmpStationaryPoint(bool on = true){
        isZmpStationaryPoint_ = on;
    }
            
    bool isZmpStationaryPoint() const {
        return isZmpStationaryPoint_;
    }

private:

    std::vector<JointInfo> jointInfos;
    LinkInfoMap ikLinks;
    LinkInfoMap::iterator baseLinkIter;
    Vector3 zmp_;
    bool isZmpValid_;
    bool isZmpStationaryPoint_;
    void initializeMembers();
};

typedef ref_ptr<Pose> PosePtr;
}

#endif
