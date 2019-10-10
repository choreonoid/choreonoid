/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_POSITION_VIEW_H
#define CNOID_BODYPLUGIN_POSITION_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PositionView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static PositionView* instance();

    PositionView();
    virtual ~PositionView();

    void setCoordinateFrameLabels(const char* baseFrameLabel, const char* localFrameLabel);

protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
