#ifndef CNOID_BASE_NEW_ITEM_TREE_VIEW_H
#define CNOID_BASE_NEW_ITEM_TREE_VIEW_H

#include "View.h"

namespace cnoid {

class NewItemTreeView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    NewItemTreeView();
    ~NewItemTreeView();

protected:
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
