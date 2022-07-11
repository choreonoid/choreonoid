/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_LINK_PROPERTY_VIEW_H
#define CNOID_BODY_PLUGIN_LINK_PROPERTY_VIEW_H

#include <cnoid/View>

namespace cnoid {

class LinkPropertyView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    LinkPropertyView();
    ~LinkPropertyView();

protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;
    virtual void keyPressEvent(QKeyEvent* event) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
