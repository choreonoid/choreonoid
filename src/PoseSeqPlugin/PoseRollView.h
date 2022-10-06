#ifndef CNOID_POSE_SEQ_PLUGIN_POSE_ROLL_VIEW_H
#define CNOID_POSE_SEQ_PLUGIN_POSE_ROLL_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class PoseSeqItem;
        
class CNOID_EXPORT PoseRollView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    PoseRollView();
    ~PoseRollView();

    PoseSeqItem* currentPoseSeqItem();
    double currentTime() const;
    SignalProxy<void(double time)> sigCurrentTimeChanged();
    bool isTimeBarSyncEnabled() const;
    SignalProxy<void(bool on)> sigTimeBarSyncToggled();

protected:
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;
    virtual bool eventFilter(QObject *obj, QEvent *event) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
