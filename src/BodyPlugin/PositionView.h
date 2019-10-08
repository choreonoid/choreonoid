/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_POSITION_VIEW_H
#define CNOID_BODYPLUGIN_POSITION_VIEW_H

#include <cnoid/View>

namespace cnoid {

class PositionView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static PositionView* instance();

    PositionView();
    virtual ~PositionView();

protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

    class Impl;
    Impl* impl;
};

}

#endif
