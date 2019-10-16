#ifndef CNOID_BODY_PLUGIN_COORDINATE_FRAME_LIST_VIEW_H
#define CNOID_BODY_PLUGIN_COORDINATE_FRAME_LIST_VIEW_H

#include <cnoid/View>

class QModelIndex;

namespace cnoid {

class CoordinateFrameListView : public View
{
    Q_OBJECT
    
public:
    static void initializeClass(ExtensionManager* ext);

    CoordinateFrameListView();
    virtual ~CoordinateFrameListView();
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

    class Impl;
    
private Q_SLOTS:
    void onTableItemClicked(const QModelIndex& index);

private:
    Impl* impl;
};

}

#endif
