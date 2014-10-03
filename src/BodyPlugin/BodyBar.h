/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_BODY_BAR_H
#define CNOID_BODY_PLUGIN_BODY_BAR_H

#include "BodyItem.h"
#include <cnoid/ItemList>
#include <cnoid/ToolBar>
#include <cnoid/Signal>
#include <cnoid/SpinBox>
#include "exportdecl.h"

namespace cnoid {

class MessageView;
    
class CNOID_EXPORT BodyBar : public ToolBar
{
public:
    static BodyBar* instance();

    virtual ~BodyBar();

    Signal<void(const ItemList<BodyItem>& selectedBodyItems)>& sigBodyItemSelectionChanged() {
        return sigBodyItemSelectionChanged_;
    }

    SignalProxy<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged() {
        return sigCurrentBodyItemChanged_;
    }

    const ItemList<BodyItem>& selectedBodyItems() {
        return selectedBodyItems_;
    }

    BodyItem* currentBodyItem() {
        return currentBodyItem_.get();
    }

    bool makeSingleSelection(BodyItemPtr bodyItem);

protected:

    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);

private:

    BodyBar();

    MessageView& mes;

    BodyItemPtr currentBodyItem_;
    ItemList<BodyItem> selectedBodyItems_;
    ItemList<BodyItem> targetBodyItems;

    DoubleSpinBox* stanceWidthSpin;

    Connection connectionOfItemSelectionChanged;
    Connection connectionOfCurrentBodyItemDetachedFromRoot;
            
    Signal<void(const ItemList<BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
    Signal<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;

    void onItemSelectionChanged(const ItemList<BodyItem>& bodyItems);
    void onBodyItemDetachedFromRoot();
    void onCopyButtonClicked();
    void onPasteButtonClicked();
    void onOriginButtonClicked();
    void onPoseButtonClicked(BodyItem::PresetPoseID id);
    void onSymmetricCopyButtonClicked(int direction, bool doMirrorCopy);
    void moveCM(BodyItem::PositionType position);
    void setZmp(BodyItem::PositionType position);
    void setStance();
};

}

#endif
