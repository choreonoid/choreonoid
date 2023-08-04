#ifndef CNOID_POSE_SEQ_PLUGIN_BODY_KEY_POSE_H
#define CNOID_POSE_SEQ_PLUGIN_BODY_KEY_POSE_H

#include "AbstractPose.h"
#include <cnoid/EigenTypes>
#include <vector>
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

    void setNumJoints(int n);
            
    int numJoints() const {
        return jointInfos.size();
    }

    void setJointDisplacement(int jointId, double q);

    double jointDisplacement(int jointId) const {
        return jointInfos[jointId].q;
    }

    bool isJointValid(int jointId) const {
        if(jointId < 0 || jointId >= (int)jointInfos.size()){
            return false;
        }
        return jointInfos[jointId].isValid;
    }

    void setJointStationaryPoint(int jointId, bool on = true);

    bool isJointStationaryPoint(int jointId) const {
        if(jointId >= (int)jointInfos.size()){
            return false;
        }
        return jointInfos[jointId].isStationaryPoint;
    }

    bool invalidateJoint(int jointId);

    class LinkInfo
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        LinkInfo();

        Isometry3& T() { return T_; }
        const Isometry3& T() const { return T_; }
        Isometry3& position() { return T_; }
        const Isometry3& position() const { return T_; }
        template<class Scalar, int Mode, int Options>
        void setPosition(const Eigen::Transform<Scalar, 3, Mode, Options>& T){
            T_ = T.template cast<Isometry3::Scalar>();
        }
        template<class Derived>
        void setPosition(const Eigen::MatrixBase<Derived>& T){
            T_ = T.template cast<Isometry3::Scalar>();
        }
        template<typename Derived1, typename Derived2>
        void setPosition(const Eigen::MatrixBase<Derived1>& rotation, const Eigen::MatrixBase<Derived2>& translation){
            T_.linear() = rotation;
            T_.translation() = translation;
        }
        Isometry3::TranslationPart p() { return T_.translation(); }
        Isometry3::ConstTranslationPart p() const { return T_.translation(); }
        Isometry3::TranslationPart translation() { return T_.translation(); }
        Isometry3::ConstTranslationPart translation() const { return T_.translation(); }
        template<typename Derived>
        void setTranslation(const Eigen::MatrixBase<Derived>& p) {
            T_.translation() = p.template cast<Isometry3::Scalar>();
        }
        Isometry3::LinearPart R() { return T_.linear(); }
        Isometry3::ConstLinearPart R() const { return T_.linear(); }
        Isometry3::LinearPart rotation() { return T_.linear(); }
        Isometry3::ConstLinearPart rotation() const { return T_.linear(); }
        template<typename Derived>
        void setRotation(const Eigen::MatrixBase<Derived>& R) {
            T_.linear() = R.template cast<Isometry3::Scalar>();
        }
        template<typename T>
        void setRotation(const Eigen::AngleAxis<T>& a) {
            T_.linear() = a.template cast<Isometry3::Scalar>().toRotationMatrix();
        }
        template<typename Derived>
        void setRotation(const Eigen::QuaternionBase<Derived>& q) {
            T_.linear() = q.template cast<Isometry3::Scalar>().toRotationMatrix();
        }
        
        bool isBaseLink() const { return isBaseLink_; }
        void setStationaryPoint(bool on){ isStationaryPoint_ = on; }
        bool isStationaryPoint() const { return isStationaryPoint_; }
        bool isTouching() const { return isTouching_; }
        const Vector3& partingDirection() const { return partingDirection_; }
        const std::vector<Vector3>& contactPoints() const { return contactPoints_; }
        void setTouching(const Vector3& partingDirection, const std::vector<Vector3>& contactPoints);
        void setTouching(bool on = true);
        void clearTouching();
        bool isSlave() const { return isSlave_; }
        void setSlave(bool on) { isSlave_ = on; }
                
    private:
        Isometry3 T_;
        bool isBaseLink_;
        bool isStationaryPoint_;
        bool isTouching_;
        bool isSlave_;
        Vector3 partingDirection_;
        std::vector<Vector3> contactPoints_;
        friend class BodyKeyPose;
    };

    typedef std::map<
        int, LinkInfo, std::less<int>, 
        Eigen::aligned_allocator<std::pair<const int, LinkInfo>>> LinkInfoMap;

    void clearIkLinks();

    size_t numIkLinks(){
        return ikLinks.size();
    }

    LinkInfo* getOrCreateIkLink(int linkIndex){
        return &ikLinks[linkIndex];
    }
    
    bool removeIkLink(int linkIndex);

    const LinkInfo* ikLinkInfo(int linkIndex) const {
        LinkInfoMap::const_iterator p = ikLinks.find(linkIndex);
        return (p != ikLinks.end()) ? &p->second : nullptr;
    }

    LinkInfo* ikLinkInfo(int linkIndex) {
        LinkInfoMap::iterator p = ikLinks.find(linkIndex);
        return (p != ikLinks.end()) ? &p->second : nullptr;
    }
            
    LinkInfoMap::iterator ikLinkBegin() { return ikLinks.begin(); }
    const LinkInfoMap::const_iterator ikLinkBegin() const { return ikLinks.begin(); }
    LinkInfoMap::iterator ikLinkEnd() { return ikLinks.end(); }
    const LinkInfoMap::const_iterator ikLinkEnd() const { return ikLinks.end(); }

    LinkInfo* setBaseLink(int linkIndex);
    LinkInfo* setBaseLink(int linkIndex, const Isometry3& position);
            
    int baseLinkIndex() const {
        return (baseLinkIter != ikLinks.end()) ? baseLinkIter->first : -1;
    }
            
    LinkInfo* baseLinkInfo() {
        return (baseLinkIter != ikLinks.end()) ? &baseLinkIter->second : nullptr;
    }

    void invalidateBaseLink();

    const Vector3 zmp() const {
        return zmp_;
    }

    bool isZmpValid() const {
        return isZmpValid_;
    }
            
    bool isZmpStationaryPoint() const {
        return isZmpStationaryPoint_;
    }

    void setZmp(const Vector3& p);
    void setZmpStationaryPoint(bool on = true);
    bool invalidateZmp();
            
    bool hasSameParts(AbstractPose* pose) const override;
    bool restore(const Mapping& archive, const Body* body) override;
    void store(Mapping& archive, const Body* body) const override;

    /**
       This is temporarily defined to enable or disable outputting contact points in the store function.
       In the future, the signature of the store function should be modified so that this kind of global
       function can be avoided.
    */
    static void setContactPointOutputEnabled(bool on);
    static bool isContactPointOutputEnabled();

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
