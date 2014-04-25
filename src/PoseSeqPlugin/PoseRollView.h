/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_POSE_SEQ_PLUGIN_POSE_ROLL_VIEW_H_INCLUDED
#define CNOID_POSE_SEQ_PLUGIN_POSE_ROLL_VIEW_H_INCLUDED

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class Archive;
class PoseRollViewImpl;
class PoseSeqItem;
        
class CNOID_EXPORT PoseRollView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    PoseRollView();
    ~PoseRollView();

    PoseSeqItem* currentPoseSeqItem();
        
private:
    PoseRollViewImpl* impl;
        
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
    virtual bool eventFilter(QObject *obj, QEvent *event);
        
};
}

#endif
