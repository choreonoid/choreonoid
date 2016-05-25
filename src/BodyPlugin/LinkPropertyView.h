/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_LINK_PROPERTY_VIEW_H
#define CNOID_BODY_PLUGIN_LINK_PROPERTY_VIEW_H

#include <cnoid/View>

namespace cnoid {

class LinkPropertyViewImpl;

class LinkPropertyView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    LinkPropertyView();
    ~LinkPropertyView();

protected:
    void keyPressEvent(QKeyEvent* event);

private:
    LinkPropertyViewImpl* impl;
};

}

#endif
