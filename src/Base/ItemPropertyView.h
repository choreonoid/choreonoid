#ifndef CNOID_BASE_ITEM_PROPERTY_VIEW_H
#define CNOID_BASE_ITEM_PROPERTY_VIEW_H

#include <cnoid/View>

namespace cnoid {

class ItemPropertyView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    ItemPropertyView();
    ~ItemPropertyView();

    virtual void onAttachedMenuRequest(MenuManager& menuManager);

private:
    class Impl;
    Impl* impl;
};

}

#endif
