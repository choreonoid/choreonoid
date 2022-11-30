#ifndef CNOID_BODY_PLUGIN_LINK_GRAPH_VIEW_H
#define CNOID_BODY_PLUGIN_LINK_GRAPH_VIEW_H

#include "BodyPositionGraphViewBase.h"
#include <cnoid/Buttons>
#include <QBoxLayout>

namespace cnoid {

class LinkGraphView : public BodyPositionGraphViewBase
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    LinkGraphView();

protected:
    virtual void addTrajectory(ItemInfo& info, Link* link, std::shared_ptr<BodyPositionSeq> seq) override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;
            
private:
    void setupElementToggleSet(QBoxLayout* box, ToggleToolButton toggles[], const char* labels[], bool isActive);
    void onDataRequest(
        ItemInfo& info, int linkIndex, int type, int axis, int frameIndex0, int size, double* out_values);
    void onDataModified(
        ItemInfo& info, int linkIndex, int type, int axis, int frameIndex0, int size, double* values);

    ToggleToolButton xyzToggles[3];
    ToggleToolButton rpyToggles[3];
    ConnectionSet toggleConnections;
};

}

#endif
