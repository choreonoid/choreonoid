#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_BODY_ITEM_UTIL_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_BODY_ITEM_UTIL_H

#include "exportdecl.h"

namespace cnoid {

class KinematicBodyItemSet;
class MprPosition;
class MessageOut;

CNOID_EXPORT bool applyPosition(
    KinematicBodyItemSet* bodyItemSet, MprPosition* position, bool doNotify, MessageOut* mout);
CNOID_EXPORT bool superimposePosition(KinematicBodyItemSet* bodyItemSet, MprPosition* position);
CNOID_EXPORT void clearSuperimposition(KinematicBodyItemSet* bodyItemSet);
CNOID_EXPORT bool touchupPosition(KinematicBodyItemSet* bodyItemSet, MprPosition* position, MessageOut* mout);

}

#endif
