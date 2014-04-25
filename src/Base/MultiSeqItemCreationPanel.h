/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_GUIBASE_MULTI_SEQ_ITEM_CREATION_PANEL_H_INCLUDED
#define CNOID_GUIBASE_MULTI_SEQ_ITEM_CREATION_PANEL_H_INCLUDED

#include "SpinBox.h"
#include <cnoid/ItemManager>
#include <QLineEdit>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MultiSeqItemCreationPanel : public ItemCreationPanel
{
public:
    MultiSeqItemCreationPanel(const QString& numSeqsCaption);
        
    virtual bool initializePanel(Item* protoItem);
    virtual bool initializeItem(Item* protoItem);
        
private:
    QLineEdit* nameEntry;
    SpinBox* numSeqsSpin;
    DoubleSpinBox* timeLengthSpin;
    DoubleSpinBox* frameRateSpin;
};
}


#endif
