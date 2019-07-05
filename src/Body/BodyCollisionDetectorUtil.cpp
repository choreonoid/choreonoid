/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BodyCollisionDetectorUtil.h"
#include <cnoid/ValueTree>
#include <vector>

using namespace std;
using namespace cnoid;

namespace {

void setStaticFlags(Link* link, vector<bool>& staticFlags)
{
    staticFlags[link->index()] = true;
    for(Link* child = link->child(); child; child = child->sibling()){
        if(child->isFixedJoint()){
            setStaticFlags(child, staticFlags);
        }
    }
}

}


/**
   @return top of the geometry indices assigned to the body.
   The geometry id corresponding to a link is <the top index> + <link->index()>.
*/
int cnoid::addBodyToCollisionDetector(Body& body, CollisionDetector& detector, bool enableSelfCollisions)
{
    const int idTop = detector.numGeometries();
    const int numLinks = body.numLinks();
    int excludeTreeDepth = 1;
    vector<bool> exclusions(numLinks, false);
    vector<bool> staticFlags(numLinks, false);
    vector<vector<int>> excludeLinkGroups;
    
    const Mapping& cdInfo = *body.info()->findMapping("collisionDetection");
    if(cdInfo.isValid()){
        excludeTreeDepth = cdInfo.get("excludeTreeDepth", excludeTreeDepth);
        const Listing& excludeLinks = *cdInfo.findListing("excludeLinks");
        for(int i=0; i < excludeLinks.size(); ++i){
            Link* link = body.link(excludeLinks[i].toString());
            if(link){
                exclusions[link->index()] = true;
            }
        }

        const Listing& excludeLinkGroupList = *cdInfo.findListing("excludeLinkGroups");
        excludeLinkGroups.resize(excludeLinkGroupList.size());
        for(int i=0; i < excludeLinkGroupList.size(); ++i){
            const Mapping&  groupInfo = *excludeLinkGroupList[i].toMapping();
            if(groupInfo.isValid()){
                string groupName = groupInfo.get("name");
                const Listing& excludeLinks = *groupInfo.findListing("links");
                vector<int>& excludeLinkGroup = excludeLinkGroups[i];
                for(int j=0; j < excludeLinks.size(); ++j){
                    Link* link = body.link(excludeLinks[j].toString());
                    if(link){
                        excludeLinkGroup.push_back(link->index());
                    }
                }
            }
        }
    }

    if(body.isStaticModel() || body.rootLink()->isFixedJoint()){
        setStaticFlags(body.rootLink(), staticFlags);
    }
    for(int i=0; i < numLinks; ++i){
        if(exclusions[i]){
            detector.addGeometry(0);
        } else {
            auto handle = detector.addGeometry(body.link(i)->collisionShape());
            if(handle && staticFlags[i]){
                detector.setGeometryStatic(*handle);
            }
        }
    }

    if(!enableSelfCollisions){
        // exclude all the self link pairs
        for(int i=0; i < numLinks; ++i){
            if(!exclusions[i]){
                for(int j=i+1; j < numLinks; ++j){
                    if(!exclusions[j]){
                        detector.setNonInterfarenceGeometyrPair(i + idTop, j + idTop);
                    }
                }
            }
        }
    } else {
        // exclude the link pairs whose distance in the tree is less than excludeTreeDepth
        for(int i=0; i < numLinks; ++i){
            if(!exclusions[i]){
                Link* link1 = body.link(i);
                for(int j=i+1; j < numLinks; ++j){
                    if(!exclusions[j]){
                        Link* link2 = body.link(j);
                        Link* parent1 = link1;
                        Link* parent2 = link2;
                        for(int k=0; k < excludeTreeDepth; ++k){
                            if(parent1){
                                parent1 = parent1->parent();
                            }
                            if(parent2){
                                parent2 = parent2->parent();
                            }
                            if(!parent1 && !parent2){
                                break;
                            }
                            if(parent1 == link2 || parent2 == link1){
                                detector.setNonInterfarenceGeometyrPair(i + idTop, j + idTop);
                            }
                        }
                    }
                }
            }
        }

        for(int i=0; i<excludeLinkGroups.size(); i++ ){
            vector<int>& excludeLinkGroup = excludeLinkGroups[i];
            for(int j=0; j<excludeLinkGroup.size(); j++){
                int index1=excludeLinkGroup[j];
                for(int k=j+1; k<excludeLinkGroup.size(); k++){
                    int index2=excludeLinkGroup[k];
                    if(!exclusions[index1] && !exclusions[index2]){
                        detector.setNonInterfarenceGeometyrPair(index1 + idTop, index2 + idTop);
                    }
                }
            }
        }
    }

    return idTop;
}
