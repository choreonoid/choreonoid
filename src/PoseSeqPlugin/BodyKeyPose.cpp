#include "BodyKeyPose.h"
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/EigenArchive>

using namespace std;
using namespace cnoid;

namespace {

bool isContactPointOutputEnabled_ = false;

}


BodyKeyPose::BodyKeyPose()
{
    initializeMembers();
}
    

BodyKeyPose::BodyKeyPose(int numJoints)
    : jointInfos(numJoints)
{
    initializeMembers();
}


BodyKeyPose::BodyKeyPose(const BodyKeyPose& org)
    : AbstractPose(org),
      jointInfos(org.jointInfos),
      ikLinks(org.ikLinks)
{
    zmp_ = org.zmp_;
    isZmpValid_ = org.isZmpValid_;
    isZmpStationaryPoint_ = org.isZmpStationaryPoint_;

    baseLinkIter = ikLinks.end();
    if(org.baseLinkIter != org.ikLinks.end()){
        int baseLinkIndex = org.baseLinkIter->first;
        baseLinkIter = ikLinks.find(baseLinkIndex);
    }
}


void BodyKeyPose::initializeMembers()
{
    baseLinkIter = ikLinks.end();
    isZmpValid_ = false;
    isZmpStationaryPoint_ = true;
}


BodyKeyPose::~BodyKeyPose()
{

}


Referenced* BodyKeyPose::doClone(CloneMap*) const
{
    return new BodyKeyPose(*this);
}


void BodyKeyPose::setNumJoints(int n)
{
    jointInfos.resize(n);
}


void BodyKeyPose::setJointDisplacement(int jointId, double q)
{
    if(jointId >= 0){
        if(jointId >= (int)jointInfos.size()){
            setNumJoints(jointId + 1);
        }
        JointInfo& info = jointInfos[jointId];
        info.q = q;
        info.isValid = true;
    }
}


void BodyKeyPose::setJointStationaryPoint(int jointId, bool on)
{
    if(jointId >= (int)jointInfos.size()){
        setNumJoints(jointId + 1);
    }
    jointInfos[jointId].isStationaryPoint = on;
}


bool BodyKeyPose::invalidateJoint(int jointId)
{
    if(jointId < (int)jointInfos.size()){
        if(jointInfos[jointId].isValid){
            jointInfos[jointId].isValid = false;
            return true;
        }
    }
    return false;
}


bool BodyKeyPose::hasSameParts(AbstractPose* unit) const
{
    auto pose = dynamic_cast<BodyKeyPose*>(unit);
    if(!pose){
        return false;
    }
    const int n = numJoints();
    if(n != pose->numJoints()){
        return false;
    }
    for(int i=0; i < n; ++i){
        if(isJointValid(i) != pose->isJointValid(i)){
            return false;
        }
    }
    return true;
}


bool BodyKeyPose::empty() const
{
    if(!ikLinks.empty()){
        return false;
    }
    if(isZmpValid_){
        return false;
    }
    for(size_t i=0; i < jointInfos.size(); ++i){
        if(jointInfos[i].isValid){
            return false;
        }
    }
    return true;
}
            

void BodyKeyPose::clear()
{
    jointInfos.clear();
    ikLinks.clear();
    initializeMembers();
}


void BodyKeyPose::clearIkLinks()
{
    ikLinks.clear();
    baseLinkIter = ikLinks.end();
}


bool BodyKeyPose::removeIkLink(int linkIndex)
{
    LinkInfoMap::iterator p = ikLinks.find(linkIndex);
    if(p != ikLinks.end()){
        if(p == baseLinkIter){
            baseLinkIter = ikLinks.end();
        }
        ikLinks.erase(p);
        return true;
    }
    return false;
}


BodyKeyPose::LinkInfo* BodyKeyPose::setBaseLink(int linkIndex)
{
    if(baseLinkIter != ikLinks.end()){
        const int oldIndex = baseLinkIter->first;
        if(linkIndex == oldIndex){
            return &baseLinkIter->second;
        }
        baseLinkIter->second.isBaseLink_ = false;
    }
    baseLinkIter = ikLinks.insert(make_pair(linkIndex, LinkInfo())).first;
    LinkInfo& info = baseLinkIter->second;
    info.isBaseLink_ = true;

    return &info;
}


BodyKeyPose::LinkInfo* BodyKeyPose::setBaseLink(int linkIndex, const Isometry3& position)
{
    auto info = setBaseLink(linkIndex);
    if(info){
        info->setPosition(position);
    }
    return info;
}


void BodyKeyPose::invalidateBaseLink()
{
    if(baseLinkIter != ikLinks.end()){
        baseLinkIter->second.isBaseLink_ = false;
        baseLinkIter = ikLinks.end();
    }
}


void BodyKeyPose::setZmp(const Vector3& p)
{
    isZmpValid_ = true;
    zmp_ = p;
}


bool BodyKeyPose::invalidateZmp()
{
    bool ret = isZmpValid_;
    isZmpValid_ = false;
    return ret;
}


void BodyKeyPose::setZmpStationaryPoint(bool on)
{
    isZmpStationaryPoint_ = on;
}



bool BodyKeyPose::restore(const Mapping& archive, const Body* body)
{
    clear();
    
    const Listing& jointIndices = *archive.findListing("joints");
    if(jointIndices.isValid()){
        int maxIndex = jointIndices.back()->toInt();
        setNumJoints(maxIndex + 1);
        const Listing& qs = *archive["q"].toListing();
        for(int i=0; i < jointIndices.size(); ++i){
            setJointDisplacement(jointIndices[i].toInt(), qs[i].toDouble());
        }
    }
    const Listing& stationaryPoints = *archive.findListing("spJoints");
    if(stationaryPoints.isValid()){
        for(int i=0; i < stationaryPoints.size(); ++i){
            jointInfos[stationaryPoints[i].toInt()].isStationaryPoint = true;
        }
    }

    const Listing& ikLinkNodes = *archive.findListing("ikLinks");
    if(ikLinkNodes.isValid()){
        for(int i=0; i < ikLinkNodes.size(); ++i){
            const Mapping& ikLinkNode = *ikLinkNodes[i].toMapping();
            int index = -1;
            ValueNode* nameNode = ikLinkNode.find("name");
            if(nameNode->isValid()){
                Link* link = body->link(nameNode->toString());
                if(link){
                    index = link->index();
                }
            }
            if(index < 0){
                ValueNode* indexNode = ikLinkNode.find("index");
                if(indexNode->isValid()){
                    index = indexNode->toInt();
                }
            }
            if(index >= 0){
                Vector3 p;
                Matrix3 R;
                if(read(ikLinkNode, "translation", p) && read(ikLinkNode, "rotation", R)){
                    LinkInfo* info = getOrCreateIkLink(index);
                    info->setTranslation(p);
                    info->setRotation(R);
                    info->setStationaryPoint(ikLinkNode.get("isStationaryPoint", false));
                    if(ikLinkNode.get("isBaseLink", false)){
                        setBaseLink(index);
                    }
                    if(ikLinkNode.get("isTouching", false)){
                        Vector3 partingDirection(0.0, 0.0, 1.0);
                        read(ikLinkNode, "partingDirection", partingDirection);
                        vector<Vector3> contactPoints;
                        if(auto& contactPointNodes = *ikLinkNode.findListing("contactPoints")){
                            int n = contactPointNodes.size();
                            contactPoints.resize(n);
                            for(int j=0; j < n; ++j){
                                readEx(contactPointNodes[j].toListing(), contactPoints[j]);
                            }
                        }
                        info->setTouching(partingDirection, contactPoints);
                    }
                    info->setSlave(ikLinkNode.get("isSlave", false));
                }
            }
        }
    }

    if(read(archive, "zmp", zmp_)){
        isZmpValid_ = true;
        archive.read("isZmpStationaryPoint", isZmpStationaryPoint_);
    }
    
    return true;
}


void BodyKeyPose::store(Mapping& archive, const Body* body) const
{
    archive.write("type", "Pose");

    // For keeping the compatibility of the pose seq file
    archive.write("name", "", DOUBLE_QUOTED);

    ListingPtr jointIndices = new Listing();
    ListingPtr qs = new Listing();
    qs->setFloatingNumberFormat(archive.floatingNumberFormat());
    ListingPtr spJoints = new Listing();

    int n = numJoints();
    for(int i=0; i < n; ++i){
        const JointInfo& info = jointInfos[i];
        if(info.isValid){
            jointIndices->append(i, 10, n);
            qs->append(info.q, 10, n);
            if(info.isStationaryPoint){
                spJoints->append(i, 10);
            }
        }
    }
    if(!jointIndices->empty()){
        jointIndices->setFlowStyle();
        archive.insert("joints", jointIndices);
        qs->setFlowStyle();
        archive.insert("q", qs);
        if(!spJoints->empty()){
            spJoints->setFlowStyle();
            archive.insert("spJoints", spJoints);
        }
    }

    if(!ikLinks.empty()){
        Listing& ikLinkNodes = *archive.createListing("ikLinks");
        for(LinkInfoMap::const_iterator p = ikLinks.begin(); p != ikLinks.end(); ++p){
            const int index = p->first;
            const LinkInfo& info = p->second;
            Mapping& ikLinkNode = *ikLinkNodes.newMapping();

            ikLinkNode.write("name", body->link(index)->name());
            
            ikLinkNode.write("index", index);
            if(info.isBaseLink()){
                ikLinkNode.write("isBaseLink", info.isBaseLink());
            }
            if(info.isStationaryPoint()){
                ikLinkNode.write("isStationaryPoint", info.isStationaryPoint());
            }
            ikLinkNode.setFloatingNumberFormat("%.9g");
            write(ikLinkNode, "translation", Vector3(info.translation()));
            write(ikLinkNode, "rotation", Matrix3(info.rotation()));

            if(info.isTouching()){

                ikLinkNode.write("isTouching", true);
                write(ikLinkNode, "partingDirection", info.partingDirection());

                if(isContactPointOutputEnabled_){
                    auto& points = info.contactPoints();
                    if(!points.empty()){
                        auto& pointList = *ikLinkNode.createListing("contactPoints");
                        for(auto& point : points){
                            ListingPtr pointNode = new Listing;
                            pointNode->setFlowStyle();
                            for(int i=0; i < 3; ++i){
                                pointNode->append(point(i));
                            }
                            pointList.append(pointNode);
                        }
                    }
                }
            }
            
            if(info.isSlave()){
                ikLinkNode.write("isSlave", true);
            }
        }
    }
        
    if(isZmpValid()){
        write(archive, "zmp", zmp_);
        archive.write("isZmpStationaryPoint", isZmpStationaryPoint_);
    }
}


void BodyKeyPose::setContactPointOutputEnabled(bool on)
{
    isContactPointOutputEnabled_ = on;
}


bool BodyKeyPose::isContactPointOutputEnabled()
{
    return isContactPointOutputEnabled_;
}


BodyKeyPose::LinkInfo::LinkInfo()
    : isBaseLink_(false),
      isStationaryPoint_(false),
      isTouching_(false),
      isSlave_(false)
{

}
      

void BodyKeyPose::LinkInfo::setTouching(const Vector3& partingDirection, const std::vector<Vector3>& contactPoints)
{
    isTouching_ = true;
    partingDirection_ = partingDirection;
    contactPoints_ = contactPoints;
}


void BodyKeyPose::LinkInfo::setTouching(bool on)
{
    isTouching_ = on;
    partingDirection_ = Vector3::UnitZ();
    contactPoints_.clear();
}


void BodyKeyPose::LinkInfo::clearTouching()
{
    setTouching(false);
}
