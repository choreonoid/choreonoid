/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_JOINT_STATE_VIEW_H_INCLUDED
#define CNOID_BODYPLUGIN_JOINT_STATE_VIEW_H_INCLUDED

#include "BodyItem.h"
#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class JointStateViewImpl;
    
class CNOID_EXPORT JointStateView : public cnoid::View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    JointStateView();
    virtual ~JointStateView();
            
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);

private:
    JointStateViewImpl* impl;
};
}

#endif
