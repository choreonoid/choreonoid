#ifndef CNOID_BODYPLUGIN_KINEMATIC_FAULT_CHECKER_H
#define CNOID_BODYPLUGIN_KINEMATIC_FAULT_CHECKER_H

#include <ostream>
#include <limits>
#include "exportdecl.h"

namespace cnoid {
    
class ExtensionManager;
class BodyItem;
class BodyMotionItem;

class CNOID_EXPORT KinematicFaultChecker
{
public:
    static void initializeClass(ExtensionManager* ext);
    static KinematicFaultChecker* instance();
            
    KinematicFaultChecker();
    virtual ~KinematicFaultChecker();

    int checkFaults(
        BodyItem* bodyItem, BodyMotionItem* motionItem,
        double beginningTime = 0.0, double endingTime = std::numeric_limits<double>::max());

private:
    class Impl;
    Impl* impl;
};

}

#endif

