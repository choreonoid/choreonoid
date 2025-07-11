#include "MprBodyItemUtil.h"
#include "MprPosition.h"
#include <cnoid/BodyItem>
#include <cnoid/BodySuperimposerAddon>
#include <cnoid/BodyItemKinematicsKit>
#include <cnoid/KinematicBodyItemSet>
#include <cnoid/MessageOut>
#include <cnoid/Format>
#include "gettext.h"

using namespace std;
using namespace cnoid;


bool cnoid::applyPosition
(KinematicBodyItemSet* bodyItemSet, MprPosition* position, bool doNotify, MessageOut* mout)
{
    bool updated = false;

    string errorMessage;

    auto composite = position->castOrConvertToCompositePosition(bodyItemSet);

    for(auto index : composite->findMatchedPositionIndices(bodyItemSet)){
        auto subPosition = composite->position(index);
        auto bodyPart = bodyItemSet->bodyItemPart(index);
        auto bodyItem = bodyPart->bodyItem();
        if(subPosition->apply(bodyPart)){
            updated = true;
            auto ikPosition = subPosition->ikPosition();
            if(ikPosition){
                bodyPart->setReferenceRpy(ikPosition->referenceRpy());
                bodyPart->setCurrentBaseFrame(ikPosition->baseFrameId());
                bodyPart->setCurrentOffsetFrame(ikPosition->offsetFrameId());
            }
            if(doNotify){
                bodyItem->notifyKinematicStateUpdate();
                if(ikPosition){
                    bodyPart->notifyFrameSetChange();
                }
            }
        } else {
            if(auto ikPosition = subPosition->ikPosition()){
                if(doNotify){
                    bodyPart->notifyPositionError(subPosition->ikPosition()->position());
                }
                if(mout){
                    string tcpName;
                    if(auto tcpFrame = bodyPart->offsetFrame(ikPosition->offsetFrameId())){
                        tcpName = tcpFrame->note();
                        if(tcpName.empty()){
                            auto& id = tcpFrame->id();
                            if(id.isInt()){
                                tcpName = formatR(_("TCP {0}"), id.toInt());
                            } else {
                                tcpName = id.toString();
                            }
                        }
                    }
                    if(tcpName.empty()){
                        tcpName = "TCP";
                    }
                    if(!errorMessage.empty()){
                        errorMessage += "\n";
                    }
                    errorMessage +=
                        formatR(_("{0} of {1} cannot be moved to position {2}."),
                                tcpName, bodyItem->displayName(), position->id().label());
                }
            }
        }
    }

    if(!errorMessage.empty() && mout){
        mout->putErrorln(errorMessage);
    }

    return updated;
}


namespace {

bool forEachToplevelBodyItem
(KinematicBodyItemSet* bodyItemSet, std::function<bool(BodyItem* bodyItem)> callback)
{
    bool result = false;
    
    // Extract the top-level parent bodies 
    std::map<Body*, BodyItem*> topBodyMap;
    for(auto index : bodyItemSet->validBodyPartIndices()){
        auto bodyItem = bodyItemSet->bodyItem(index);
        auto body = bodyItem->body();
        topBodyMap.insert(std::pair<Body*, BodyItem*>(body, bodyItem));
    }
    auto it = topBodyMap.begin();
    while(it != topBodyMap.end()){
        bool erased = false;
        auto body = it->first;
        while((body = body->parentBody())){
            if(topBodyMap.find(body) != topBodyMap.end()){
                it = topBodyMap.erase(it);
                erased = true;
                break;
            }
        }
        if(!erased){
            ++it;
        }
    }

    for(auto& kv : topBodyMap){
        auto bodyItem = kv.second;
        if(callback(bodyItem)){
            result = true;
        }
    }

    return result;
}

bool superimposePosition(KinematicBodyItemSet* bodyItemSet, BodyItem* bodyItem, MprCompositePosition* composite)
{
    bool result = false;
    
    if(auto superimposer = bodyItem->getAddon<BodySuperimposerAddon>()){
        auto targetBody = bodyItem->body();
        bool updated = superimposer->updateSuperimposition(
            [bodyItemSet, composite, targetBody](){
                // Update the positions of a top-level parent body and its child bodies
                bool applied = false;
                for(auto index : bodyItemSet->validBodyPartIndices()){
                    if(auto position = composite->position(index)){
                        auto kinematicsKit = bodyItemSet->bodyPart(index);
                        auto body = kinematicsKit->body();
                        bool doApply = false;
                        auto tmpBody = body;
                        while(tmpBody){
                            if(tmpBody == targetBody){
                                doApply = true;
                                break;
                            }
                            tmpBody = tmpBody->parentBody();
                        }
                        if(doApply && position->apply(kinematicsKit)){
                            applied = true;
                        }
                    }
                }
                return applied;
            });

        if(updated){
            result = true;
        }
    }

    return result;
}

}

/**
   \todo Simplify the following implementation by using an independent BodySuperimposerAddons
   for each body item and updating the positions of all the boides simultaneously.
*/
bool cnoid::superimposePosition(KinematicBodyItemSet* bodyItemSet, MprPosition* position)
{
    MprCompositePositionPtr composite = position->castOrConvertToCompositePosition(bodyItemSet);
    return forEachToplevelBodyItem(
        bodyItemSet,
        [bodyItemSet, composite](BodyItem* bodyItem){
            return ::superimposePosition(bodyItemSet, bodyItem, composite);
        });
}


void cnoid::clearSuperimposition(KinematicBodyItemSet* bodyItemSet)
{
    forEachToplevelBodyItem(
        bodyItemSet,
        [](BodyItem* bodyItem){
            if(auto superimposer = bodyItem->findAddon<BodySuperimposerAddon>()){
                superimposer->clearSuperimposition();
            }
            return true;
        });
}


bool cnoid::touchupPosition
(KinematicBodyItemSet* bodyItemSet, MprPosition* position, MessageOut* mout)
{
    bool result = position->fetch(bodyItemSet, mout);
    if(result){
        position->notifyUpdate(MprPosition::PositionUpdate);
    }
    return result;
}
