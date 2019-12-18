/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_JOINT_STATE_VIEW_H
#define CNOID_BODY_PLUGIN_JOINT_STATE_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT JointStateView : public cnoid::View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    JointStateView();
    virtual ~JointStateView();
            
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);

private:
    class Impl;
    Impl* impl;
};

}

#endif
