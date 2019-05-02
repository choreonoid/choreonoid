#ifndef CNOID_OPENRTM_PLUGIN_RTS_DIAGRAM_VIEW_EX_H
#define CNOID_OPENRTM_PLUGIN_RTS_DIAGRAM_VIEW_EX_H

#include <cnoid/View>
#include <cnoid/Dialog>

namespace cnoid {

class RTSDiagramViewExImpl;

class RTSDiagramViewEx : public View
{
    Q_OBJECT

public:
    static void initializeClass(ExtensionManager* ext);
    static RTSDiagramViewEx* instance();

    RTSDiagramViewEx();
    virtual ~RTSDiagramViewEx();

public Q_SLOTS:
    void onRTSCompSelectionChange();

protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    RTSDiagramViewExImpl* impl;
};

}

#endif 
