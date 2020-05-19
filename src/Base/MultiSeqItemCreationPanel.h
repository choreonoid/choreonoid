#ifndef CNOID_BASE_MULTI_SEQ_ITEM_CREATION_PANEL_H
#define CNOID_BASE_MULTI_SEQ_ITEM_CREATION_PANEL_H

#include "SpinBox.h"
#include <cnoid/ItemManager>
#include <QLineEdit>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MultiSeqItemCreationPanel : public ItemCreationPanel
{
public:
    MultiSeqItemCreationPanel(const QString& numSeqsCaption);
        
    virtual bool initializePanel(Item* protoItem, Item* parentItem) override;
    virtual bool initializeItem(Item* protoItem, Item* parentItem) override;
        
private:
    QLineEdit* nameEntry;
    SpinBox* numSeqsSpin;
    DoubleSpinBox* timeLengthSpin;
    DoubleSpinBox* frameRateSpin;
};

}

#endif
