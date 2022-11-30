#ifndef CNOID_BODY_PLUGIN_JOINT_GRAPH_VIEW_H
#define CNOID_BODY_PLUGIN_JOINT_GRAPH_VIEW_H

#include "BodyPositionGraphViewBase.h"

namespace cnoid {

class JointGraphView : public BodyPositionGraphViewBase
{
public:
    static void initializeClass(ExtensionManager* ext);
            
    JointGraphView();

protected:
    virtual void addTrajectory(ItemInfo& info, Link* link, std::shared_ptr<BodyPositionSeq> seq) override;

private:
    void onDataRequest(ItemInfo& info, int jointId, int frameIndex0, int size, double* out_values);
    void onDataModified(ItemInfo& info, int jointId, int frameIndex0, int size, double* values);
};

}

#endif
