#ifndef CNOID_POSE_SEQ_PLUGIN_BODY_KEY_POSE_H
#define CNOID_POSE_SEQ_PLUGIN_BODY_KEY_POSE_H

#include "AbstractPose.h"
#include <cnoid/EigenTypes>
#include <map>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Mapping;
    
class CNOID_EXPORT BodyKeyPose : public AbstractPose
{
public:
    BodyKeyPose();
    BodyKeyPose(int numJoints);
    BodyKeyPose(const BodyKeyPose& org);
            
    virtual ~BodyKeyPose();

    bool empty() const;
    void clear();

    BodyKeyPose* clone() const { return static_cast<BodyKeyPose*>(doClone(nullptr)); }

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

    class LinkInfo
    {
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
        friend class BodyKeyPose;
    };

    typedef std::map<int, LinkInfo> LinkInfoMap;

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

    bool hasSameParts(AbstractPose* pose) const override;
    bool restore(const Mapping& archive, const Body* body) override;
    void store(Mapping& archive, const Body* body) const override;

protected:
    virtual Referenced* doClone(CloneMap*) const override;    

private:
    struct JointInfo
    {
        JointInfo() : isValid(false), isStationaryPoint(false) { }
        double q;
        bool isValid;
        bool isStationaryPoint;
    };
            
    std::vector<JointInfo> jointInfos;
    LinkInfoMap ikLinks;
    LinkInfoMap::iterator baseLinkIter;
    Vector3 zmp_;
    bool isZmpValid_;
    bool isZmpStationaryPoint_;
    
    void initializeMembers();
};

typedef ref_ptr<BodyKeyPose> BodyKeyPosePtr;

}

#endif
