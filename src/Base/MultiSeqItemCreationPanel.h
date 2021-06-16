#ifndef CNOID_BASE_MULTI_SEQ_ITEM_CREATION_PANEL_H
#define CNOID_BASE_MULTI_SEQ_ITEM_CREATION_PANEL_H

#include "AbstractSeqItem.h"
#include "SpinBox.h"
#include <cnoid/ItemManager>
#include <QLineEdit>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MultiSeqItemCreationPanel : public ItemCreationPanelBase<AbstractSeqItem>
{
public:
    MultiSeqItemCreationPanel(const char* numSeqsCaption);
        
    virtual bool initializeCreation(AbstractSeqItem* protoItem, Item* parentItem) override;
    virtual bool updateItem(AbstractSeqItem* protoItem, Item* parentItem) override;

    virtual void doExtraInitialization(AbstractSeqItem* protoItem, Item* parentItem);
    virtual void doExtraItemUpdate(AbstractSeqItem* protoItem, Item* parentItem);
        
private:
    QLineEdit* nameEntry;
    const char* numSeqsCaption;
    SpinBox* numSeqsSpin;
    DoubleSpinBox* timeLengthSpin;
    DoubleSpinBox* frameRateSpin;

    void createPanel();
};

}

#endif
