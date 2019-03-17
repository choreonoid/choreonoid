/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#include "Pose.h"
#include <cnoid/EigenArchive>
#include <cnoid/Link>

using namespace std;
using namespace cnoid;


PoseUnit::PoseUnit()
{
    owner = 0;
    seqLocalReferenceCounter = 0;
}


PoseUnit::PoseUnit(const PoseUnit& org)
    : name_(org.name_)
{
    owner = 0;
    seqLocalReferenceCounter = 0;
}


PoseUnit::~PoseUnit()
{

}


Pose::Pose()
{
    initializeMembers();
}
    

Pose::Pose(int numJoints)
    : jointInfos(numJoints)
{
    initializeMembers();
}


Pose::Pose(const Pose& org)
    : PoseUnit(org),
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


void Pose::initializeMembers()
{
    baseLinkIter = ikLinks.end();
    isZmpValid_ = false;
    isZmpStationaryPoint_ = true;
}


Pose::~Pose()
{

}


bool Pose::hasSameParts(PoseUnitPtr unit)
{
    PosePtr pose = dynamic_pointer_cast<Pose>(unit);
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


bool Pose::empty()
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
            

void Pose::clear()
{
    jointInfos.clear();
    ikLinks.clear();
    initializeMembers();
}


void Pose::clearIkLinks()
{
    ikLinks.clear();
    baseLinkIter = ikLinks.end();
}


bool Pose::removeIkLink(int linkIndex)
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


Pose::LinkInfo& Pose::setBaseLink(int linkIndex)
{
    if(baseLinkIter != ikLinks.end()){
        const int oldIndex = baseLinkIter->first;
        if(linkIndex == oldIndex){
            return baseLinkIter->second;
        }
        baseLinkIter->second.isBaseLink_ = false;
    }
    baseLinkIter = ikLinks.insert(make_pair(linkIndex, LinkInfo())).first;
    LinkInfo& info = baseLinkIter->second;
    info.isBaseLink_ = true;

    return info;
}


PoseUnit* Pose::duplicate()
{
    return new Pose(*this);
}


bool Pose::restore(const Mapping& archive, const BodyPtr body)
{
    clear();
    
    const Listing& jointIndices = *archive.findListing("joints");
    if(jointIndices.isValid()){
        int maxIndex = jointIndices.back()->toInt();
        setNumJoints(maxIndex + 1);
        const Listing& qs = *archive["q"].toListing();
        for(int i=0; i < jointIndices.size(); ++i){
            setJointPosition(jointIndices[i].toInt(), qs[i].toDouble());
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
                    LinkInfo* info = addIkLink(index);
                    info->p = p;
                    info->R = R;
                    info->setStationaryPoint(ikLinkNode.get("isStationaryPoint", false));
                    if(ikLinkNode.get("isBaseLink", false)){
                        setBaseLink(index);
                    }
                    Vector3 partingDirection;
                    if(ikLinkNode.get("isTouching", false) &&
                       read(ikLinkNode, "partingDirection", partingDirection)){
                        info->setTouching(partingDirection);
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


void Pose::store(Mapping& archive, const BodyPtr body) const
{
    archive.write("type", "Pose");
    archive.write("name", name(), DOUBLE_QUOTED);

    ListingPtr jointIndices = new Listing();
    ListingPtr qs = new Listing();
    qs->setDoubleFormat(archive.doubleFormat());
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
            write(ikLinkNode, "translation", info.p);
            write(ikLinkNode, "rotation", info.R);

            if(info.isTouching()){
                ikLinkNode.write("isTouching", true);
                write(ikLinkNode, "partingDirection", info.partingDirection());
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
