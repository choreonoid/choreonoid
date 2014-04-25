/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ITEM_PROPERTY_VIEW_H_INCLUDED
#define CNOID_BASE_ITEM_PROPERTY_VIEW_H_INCLUDED

#include <cnoid/View>

namespace cnoid {

class ItemPropertyViewImpl;

class ItemPropertyView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    ItemPropertyView();
    ~ItemPropertyView();

private:
    ItemPropertyViewImpl* impl;

    void keyPressEvent(QKeyEvent* event);
};

}

#endif
