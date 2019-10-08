/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_BODY_BAR_H
#define CNOID_BODY_PLUGIN_BODY_BAR_H

#include <cnoid/ItemList>
#include <cnoid/ToolBar>
#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;
class BodyBarImpl;
    
class CNOID_EXPORT BodyBar : public ToolBar
{
public:
    static BodyBar* instance();

    virtual ~BodyBar();

    /**
       Following functions are deprecated.
       Use BodySelectionManager instead of them.
    */
    SignalProxy<void(const ItemList<BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged();
    SignalProxy<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged();
    const ItemList<BodyItem>& selectedBodyItems();
    const ItemList<BodyItem>& targetBodyItems();
    BodyItem* currentBodyItem();
    bool makeSingleSelection(BodyItem* bodyItem);

protected:
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);

private:
    BodyBar();
    BodyBarImpl* impl;
};

}

#endif
