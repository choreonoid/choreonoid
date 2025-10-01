#include "KinematicBodySetState.h"
#include "BodyKinematicsKit.h"
#include "Body.h"
#include <utility>

using namespace cnoid;


KinematicBodySetState::KinematicBodySetState()
{

}


KinematicBodySetState::KinematicBodySetState(const KinematicBodySetState& org)
    : bodyStates(org.bodyStates)
{

}


KinematicBodySetState::KinematicBodySetState(KinematicBodySetState&& org) noexcept
    : bodyStates(std::move(org.bodyStates))
{

}


KinematicBodySetState::~KinematicBodySetState()
{

}


KinematicBodySetState& KinematicBodySetState::operator=(const KinematicBodySetState& rhs)
{
    if(this != &rhs){
        bodyStates = rhs.bodyStates;
    }
    return *this;
}


KinematicBodySetState& KinematicBodySetState::operator=(KinematicBodySetState&& rhs) noexcept
{
    if(this != &rhs){
        bodyStates = std::move(rhs.bodyStates);
    }
    return *this;
}


void KinematicBodySetState::clear()
{
    bodyStates.clear();
}


bool KinematicBodySetState::storeState(const KinematicBodySet* bodySet)
{
    if(!bodySet){
        return false;
    }

    clear();

    int maxIndex = bodySet->maxIndex();

    if(maxIndex < 0){
        return false;
    }

    bodyStates.resize(maxIndex + 1);

    for(int i = 0; i <= maxIndex; ++i){
        auto bodyPart = bodySet->bodyPart(i);
        if(bodyPart){
            auto body = bodyPart->body();
            if(body){
                // Store both position and device states
                bodyStates[i].storeStateOfBody(body);
            }
        }
    }

    return true;
}


bool KinematicBodySetState::restoreState(KinematicBodySet* bodySet) const
{
    if(!bodySet){
        return false;
    }

    int maxIndex = bodySet->maxIndex();

    // Check if the structure matches
    if(maxIndex < 0 || bodyStates.size() != maxIndex + 1){
        return false;
    }

    bool allRestored = true;

    for(int i = 0; i <= maxIndex; ++i){
        auto bodyPart = bodySet->bodyPart(i);
        if(bodyPart){
            auto body = bodyPart->body();
            if(body){
                // Restore both position and device states
                if(!bodyStates[i].restoreStateToBody(body)){
                    allRestored = false;
                }
            }
        }
    }

    return allRestored;
}