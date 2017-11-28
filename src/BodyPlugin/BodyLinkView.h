/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_BODY_LINK_VIEW_H_INCLUDED
#define CNOID_BODYPLUGIN_BODY_LINK_VIEW_H_INCLUDED

#include <cnoid/View>

namespace cnoid {

class BodyLinkViewImpl;

class BodyLinkView : public cnoid::View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static BodyLinkView* instance();

    BodyLinkView();
    virtual ~BodyLinkView();

    void switchRpyQuat(bool on);

private:
    BodyLinkViewImpl* impl;

    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
};
}

#endif
