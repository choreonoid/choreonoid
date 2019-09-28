/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_LINK_POSITION_VIEW_H
#define CNOID_BODYPLUGIN_LINK_POSITION_VIEW_H

#include <cnoid/View>

namespace cnoid {

class LinkPositionViewImpl;

class LinkPositionView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static LinkPositionView* instance();

    LinkPositionView();
    virtual ~LinkPositionView();

protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;

private:
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

    LinkPositionViewImpl* impl;
};

}

#endif
