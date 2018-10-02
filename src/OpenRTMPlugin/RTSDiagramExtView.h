/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari
 * @file
 */
#ifndef CNOID_OPENRTM_PLUGIN_RTS_DIAGRAM_EXT_VIEW_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTS_DIAGRAM_EXT_VIEW_H_INCLUDED

#include <cnoid/View>
#include <cnoid/Dialog>

using namespace cnoid;

namespace cnoid {
class RTSDiagramExtViewImpl;

class RTSDiagramExtView : public View
{
    Q_OBJECT

public:
    static void initializeClass(ExtensionManager* ext);
    static RTSDiagramExtView* instance();

    RTSDiagramExtView();
    virtual ~RTSDiagramExtView();

public Q_SLOTS:
    void onRTSCompSelectionChange();

protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    RTSDiagramExtViewImpl* impl;

};

}

#endif 
