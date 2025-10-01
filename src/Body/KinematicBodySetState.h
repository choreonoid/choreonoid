#ifndef CNOID_BODY_KINEMATIC_BODY_SET_STATE_H
#define CNOID_BODY_KINEMATIC_BODY_SET_STATE_H

#include "KinematicBodySet.h"
#include "BodyState.h"
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT KinematicBodySetState
{
public:
    KinematicBodySetState();
    KinematicBodySetState(const KinematicBodySetState& org);
    KinematicBodySetState(KinematicBodySetState&& org) noexcept;
    ~KinematicBodySetState();

    KinematicBodySetState& operator=(const KinematicBodySetState& rhs);
    KinematicBodySetState& operator=(KinematicBodySetState&& rhs) noexcept;

    void clear();
    bool empty() const { return bodyStates.empty(); }

    /**
       Store the current state of all bodies in the KinematicBodySet
    */
    bool storeState(const KinematicBodySet* bodySet);

    /**
       Restore the stored state to all bodies in the KinematicBodySet
       \return true if successful, false if the structure doesn't match
    */
    bool restoreState(KinematicBodySet* bodySet) const;

    /**
       Get the number of body states stored
    */
    int numBodyStates() const { return bodyStates.size(); }

    /**
       Access individual body state (for advanced use)
    */
    BodyState& bodyState(int index) { return bodyStates[index]; }
    const BodyState& bodyState(int index) const { return bodyStates[index]; }

private:
    std::vector<BodyState> bodyStates;
};

}

#endif