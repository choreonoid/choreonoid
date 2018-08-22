/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_RTM_IMAGE_VIEW_H
#define CNOID_OPENRTM_PLUGIN_RTM_IMAGE_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class RTMImageViewImpl;

class CNOID_EXPORT RTMImageView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    RTMImageView();
    ~RTMImageView();

    virtual void setName(const std::string& name) override;

protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;

private:
    RTMImageViewImpl* impl;
};

}

#endif
