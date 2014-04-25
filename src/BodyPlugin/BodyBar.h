/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_BODY_BAR_H_INCLUDED
#define CNOID_BODYPLUGIN_BODY_BAR_H_INCLUDED

#include "BodyItem.h"
#include <cnoid/ItemList>
#include <cnoid/ToolBar>
#include <cnoid/SignalProxy>
#include <cnoid/SpinBox>
#include "exportdecl.h"

namespace cnoid {

class MessageView;
    
class CNOID_EXPORT BodyBar : public ToolBar, public boost::signals::trackable
{
public:

    static BodyBar* instance();

    virtual ~BodyBar();

    boost::signal<void(const ItemList<BodyItem>& selectedBodyItems)>& sigBodyItemSelectionChanged() {
        return sigBodyItemSelectionChanged_;
    }

    SignalProxy< boost::signal<void(BodyItem* currentBodyItem)> > sigCurrentBodyItemChanged() {
        return sigCurrentBodyItemChanged_;
    }

    inline const ItemList<BodyItem>& selectedBodyItems() {
        return selectedBodyItems_;
    }

    inline BodyItem* currentBodyItem() {
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

    boost::signals::connection connectionOfCurrentBodyItemDetachedFromRoot;
            
    boost::signal<void(const ItemList<BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
    boost::signal<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;

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
