#include "MultiSeqItemCreationPanel.h"
#include "MultiSeqItem.h"
#include <QBoxLayout>
#include <QLabel>
#include "gettext.h"

using namespace std;
using namespace cnoid;


MultiSeqItemCreationPanel::MultiSeqItemCreationPanel(const char* numSeqsCaption)
    : nameEntry(nullptr),
      numSeqsCaption(numSeqsCaption)
{

}


void MultiSeqItemCreationPanel::createPanel()
{
    QVBoxLayout* vbox = new QVBoxLayout;
    setLayout(vbox);
        
    QHBoxLayout* hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Name")));
    nameEntry = new QLineEdit;
    hbox->addWidget(nameEntry);
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout;
    
    hbox->addWidget(new QLabel(numSeqsCaption));
    numSeqsSpin = new SpinBox;
    numSeqsSpin->setRange(1, 999);
    hbox->addWidget(numSeqsSpin);
        
    hbox->addWidget(new QLabel(_("Time length")));
    timeLengthSpin = new DoubleSpinBox;
    timeLengthSpin->setDecimals(2);
    timeLengthSpin->setRange(0.0, 9999.99);
    timeLengthSpin->setSingleStep(0.01);
    hbox->addWidget(timeLengthSpin);

    hbox->addWidget(new QLabel(_("Frame rate")));
    frameRateSpin = new DoubleSpinBox;
    frameRateSpin->setDecimals(0);
    frameRateSpin->setRange(1.0, 9999.0);
    hbox->addWidget(frameRateSpin);

    vbox->addLayout(hbox);
}
    
    
bool MultiSeqItemCreationPanel::initializeCreation(AbstractSeqItem* protoItem, Item* parentItem)
{
    if(!nameEntry){
        createPanel();
    }
    
    nameEntry->setText(protoItem->name().c_str());

    auto seq = protoItem->abstractSeq();
    double frameRate = seq->getFrameRate();
    timeLengthSpin->setValue(seq->getNumFrames() / frameRate);
    frameRateSpin->setValue(frameRate);
    
    if(auto multiSeqItem = dynamic_cast<AbstractMultiSeqItem*>(protoItem)){
        numSeqsSpin->setValue(multiSeqItem->abstractMultiSeq()->getNumParts());
    }

    doExtraInitialization(protoItem, parentItem);
    
    return true;
}


void MultiSeqItemCreationPanel::doExtraInitialization(AbstractSeqItem* /* protoItem */, Item* /* parentItem */)
{

}
    
    
bool MultiSeqItemCreationPanel::updateItem(AbstractSeqItem* protoItem, Item* parentItem)
{
    protoItem->setName(nameEntry->text().toStdString());

    auto seq = protoItem->abstractSeq();
    double frameRate = frameRateSpin->value();
    seq->setFrameRate(frameRate);
    
    if(auto multiSeqItem = dynamic_cast<AbstractMultiSeqItem*>(protoItem)){
        multiSeqItem->abstractMultiSeq()->setNumParts(numSeqsSpin->value());
    }

    doExtraItemUpdate(protoItem, parentItem);

    seq->setNumFrames(static_cast<int>(timeLengthSpin->value() * frameRate));
    
    return true;
}


void MultiSeqItemCreationPanel::doExtraItemUpdate(AbstractSeqItem* /* protoItem */, Item* /* parentItem */)
{

}

