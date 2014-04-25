/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_CHOREOGRAPHY_BODY_POSE_SEQ_VIEW_H_INCLUDED
#define CNOID_CHOREOGRAPHY_BODY_POSE_SEQ_VIEW_H_INCLUDED

#include <cnoid/View>

namespace cnoid {

class Archive;
class ExtensionManager;
class PoseSeqViewImpl;
        
class PoseSeqView : public cnoid::View
{
public:
    PoseSeqView(ExtensionManager& ext);
    ~PoseSeqView();
            
private:
    PoseSeqViewImpl* impl;

    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
};
}

#endif
