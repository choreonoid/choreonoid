/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ITEM_PROPERTY_VIEW_H
#define CNOID_BASE_ITEM_PROPERTY_VIEW_H

#include <cnoid/View>

namespace cnoid {

class ItemPropertyViewImpl;

class ItemPropertyView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    ItemPropertyView();
    ~ItemPropertyView();

    virtual void onAttachedMenuRequest(MenuManager& menuManager);

protected:
    void keyPressEvent(QKeyEvent* event);

private:
    ItemPropertyViewImpl* impl;
};

}

#endif
