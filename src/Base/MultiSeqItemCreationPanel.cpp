/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#include "MultiSeqItemCreationPanel.h"
#include "MultiSeqItem.h"
#include <QBoxLayout>
#include <QLabel>
#include "gettext.h"

using namespace std;
using namespace cnoid;


namespace {
const bool TRACE_FUNCTIONS = false;
}


MultiSeqItemCreationPanel::MultiSeqItemCreationPanel(const QString& numSeqsCaption)
{
    QVBoxLayout* vbox = new QVBoxLayout();
    setLayout(vbox);
        
    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("Name")));
    nameEntry = new QLineEdit();
    hbox->addWidget(nameEntry);
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout();
    
    hbox->addWidget(new QLabel(numSeqsCaption));
    numSeqsSpin = new SpinBox();
    numSeqsSpin->setRange(1, 999);
    hbox->addWidget(numSeqsSpin);
        
    hbox->addWidget(new QLabel(_("Time length")));
    timeLengthSpin = new DoubleSpinBox();
    timeLengthSpin->setDecimals(2);
    timeLengthSpin->setRange(0.0, 9999.99);
    timeLengthSpin->setSingleStep(0.01);
    hbox->addWidget(timeLengthSpin);

    hbox->addWidget(new QLabel(_("Frame rate")));
    frameRateSpin = new DoubleSpinBox();
    frameRateSpin->setDecimals(0);
    frameRateSpin->setRange(1.0, 9999.0);
    hbox->addWidget(frameRateSpin);

    vbox->addLayout(hbox);
}
    
    
bool MultiSeqItemCreationPanel::initializePanel(Item* protoItem)
{
    nameEntry->setText(protoItem->name().c_str());
        
    AbstractMultiSeqItem* item = dynamic_cast<AbstractMultiSeqItem*>(protoItem);
    if(item){
        auto seq = item->abstractMultiSeq();
        numSeqsSpin->setValue(seq->getNumParts());
        double frameRate = seq->getFrameRate();
        timeLengthSpin->setValue(seq->getNumFrames() / frameRate);
        frameRateSpin->setValue(frameRate);
        return true;
    }
    return false;
}
    
    
bool MultiSeqItemCreationPanel::initializeItem(Item* protoItem)
{
    protoItem->setName(nameEntry->text().toStdString());

    AbstractMultiSeqItem* item = dynamic_cast<AbstractMultiSeqItem*>(protoItem);
    if(item){
        auto seq = item->abstractMultiSeq();
        double frameRate = frameRateSpin->value();
        seq->setFrameRate(frameRate);
        seq->setNumParts(numSeqsSpin->value());
        seq->setNumFrames(static_cast<int>(timeLengthSpin->value() * frameRate));
        return true;
    }
    return false;
}
