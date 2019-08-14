/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "ScenarioView.h"
#include "ScenarioItem.h"
#include <cnoid/ViewManager>
#include <cnoid/MessageView>
#include <cnoid/RootItem>
#include <cnoid/ItemTreeView>
#include <cnoid/TimeBar>
#include <cnoid/Button>
#include <cnoid/BodyItem>
#include <cnoid/AudioItem>
#include <cnoid/MediaItem>
#include <vector>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;

namespace {
const bool TRACE_FUNCTIONS = false;
const char* emptyNameString = "--------";
}

namespace cnoid {

class MotionInfo
{
public:

    QHBoxLayout hbox;
    QLabel motionLabel;
    PushButton startButton;
    BodyMotionItemPtr motionItem;
            
    MotionInfo(BodyMotionItemPtr motionItem, bool isSequential) :
        motionItem(motionItem),
        motionLabel(motionItem->headItem()->name().c_str()) {

        startButton.setText(_("Start"));
        if(isSequential){
            startButton.setEnabled(false);
        }
        hbox.addWidget(&startButton);
        hbox.addWidget(&motionLabel);
        hbox.addStretch();
    }
};

typedef std::shared_ptr<MotionInfo> MotionInfoPtr;
            

class ScenarioViewImpl
{
public:
    ScenarioViewImpl(ScenarioView* self);
    ~ScenarioViewImpl();

    ScenarioView* self;
    ostream& os;
    TimeBar* timeBar;
    ScenarioItemPtr currentScenarioItem;
    QVBoxLayout scenarioBox;
    PushButton updateButton;
    PushButton startButton;
    QLabel currentScenarioLabel;
    CheckBox sequentialCheck;
    Connection connectionOfItemSelectionChanged;
    Connection connectionOfCurrentScenarioItemDetachedFromRoot;

    vector<MotionInfoPtr> motions;
    Connection playbackStopConnection;
            
    void onItemSelectionChanged(const ItemList<ScenarioItem>& scenarioItems);
    void setCurrentScenarioItem(ScenarioItemPtr scenarioItem);
    void update();
    void onScenarioItemDetachedFromRoot(ScenarioItemPtr worldItem);
    void start();
    void startMotion(int index);
    void onPlaybackStopped(int index);
};

}


void ScenarioView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<ScenarioView>(
        "ScenarioView", N_("Scenario"), ViewManager::SINGLE_OPTIONAL);
}


ScenarioView::ScenarioView()
{
    impl = new ScenarioViewImpl(this);
}


ScenarioViewImpl::ScenarioViewImpl(ScenarioView* self)
    : self(self),
      os(MessageView::mainInstance()->cout()),
      timeBar(TimeBar::instance())
{
    self->setDefaultLayoutArea(View::CENTER);
    
    QVBoxLayout* vbox = new QVBoxLayout();
    QHBoxLayout* hbox = new QHBoxLayout();

    updateButton.setText(_("Update"));
    updateButton.sigClicked().connect(std::bind(&ScenarioViewImpl::update, this));
    hbox->addWidget(&updateButton);

    startButton.setText(_("Start"));
    startButton.sigClicked().connect(std::bind(&ScenarioViewImpl::start, this));
    hbox->addWidget(&startButton);

    sequentialCheck.setText(_("Sequential"));
    sequentialCheck.sigToggled().connect(std::bind(&ScenarioViewImpl::update, this));
    hbox->addWidget(&sequentialCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("Scenario: ")));
    currentScenarioLabel.setText(emptyNameString);
    hbox->addWidget(&currentScenarioLabel);
    currentScenarioLabel.setAlignment(Qt::AlignLeft|Qt::AlignVCenter);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    vbox->addLayout(&scenarioBox);
    vbox->addStretch();
    self->setLayout(vbox);

    connectionOfItemSelectionChanged = 
        ItemTreeView::mainInstance()->sigSelectionChanged().connect(
            std::bind(&ScenarioViewImpl::onItemSelectionChanged, this, _1));
}


ScenarioView::~ScenarioView()
{
    delete impl;
}


ScenarioViewImpl::~ScenarioViewImpl()
{
    connectionOfItemSelectionChanged.disconnect();
    playbackStopConnection.disconnect();
}


void ScenarioViewImpl::onItemSelectionChanged(const ItemList<ScenarioItem>& scenarioItems)
{
    if(TRACE_FUNCTIONS){
        os << "ScenarioViewImpl::onItemSelectionChanged()" << endl;
    }
    
    ScenarioItemPtr scenarioItem = scenarioItems.toSingle();

    if(scenarioItem && scenarioItem != currentScenarioItem){
        setCurrentScenarioItem(scenarioItem);
    }
}
    

void ScenarioViewImpl::setCurrentScenarioItem(ScenarioItemPtr scenarioItem)
{
    if(TRACE_FUNCTIONS){
        os << "ScenarioViewImpl::setCurrentScenarioItem()" << endl;
    }
    
    connectionOfCurrentScenarioItemDetachedFromRoot.disconnect();

    currentScenarioItem = scenarioItem;

    if(scenarioItem){
        connectionOfCurrentScenarioItemDetachedFromRoot =
            scenarioItem->sigDetachedFromRoot().connect(
                std::bind(&ScenarioViewImpl::onScenarioItemDetachedFromRoot, this, scenarioItem));
    }

    update();
}


void ScenarioViewImpl::update()
{
    motions.clear();

    if(!currentScenarioItem){
        currentScenarioLabel.setText("");
    } else {
        currentScenarioLabel.setText(currentScenarioItem->name().c_str());

        ItemList<BodyMotionItem> motionItems;
        if(motionItems.extractChildItems(currentScenarioItem)){
            for(int i=0; i < motionItems.size(); ++i){
                MotionInfoPtr info(new MotionInfo(motionItems[i], sequentialCheck.isChecked()));
                scenarioBox.addLayout(&info->hbox);
                info->startButton.sigClicked().connect(std::bind(&ScenarioViewImpl::startMotion, this, i));
                motions.push_back(info);
            }
        }
    }
}


void ScenarioViewImpl::onScenarioItemDetachedFromRoot(ScenarioItemPtr scenarioItem)
{
    if(TRACE_FUNCTIONS){
        os << "ScenarioViewImpl::onScenarioItemDetachedFromRoot()" << endl;
    }
    
    if(currentScenarioItem == scenarioItem){
        setCurrentScenarioItem(0);
    }
}


void ScenarioViewImpl::start()
{
    if(TRACE_FUNCTIONS){
        os << "ScenarioViewImpl::onCollisionDetectionToggled()" << endl;
    }

    if(sequentialCheck.isChecked()){
        for(int i=0; i < motions.size(); ++i){
            motions[i]->startButton.setEnabled(i == 0);
        }
    }
}


void ScenarioViewImpl::startMotion(int index)
{
    if(index < motions.size()){
        BodyItemPtr bodyItem =  currentScenarioItem->findOwnerItem<BodyItem>();
        if(bodyItem){
            ItemTreeView* itv = ItemTreeView::mainInstance();
            MotionInfoPtr info = motions[index];

            // select the target body motion item
            ItemList<BodyMotionItem> motionItems;
            if(motionItems.extractChildItems(bodyItem)){
                for(int i=0; i < motionItems.size(); ++i){
                    BodyMotionItemPtr motionItem = motionItems[i];
                    if(itv->isItemSelected(motionItem) && (motionItem != info->motionItem)){
                        itv->selectItem(motionItem, false);
                    }
                }
            }
            itv->selectItem(info->motionItem, true);

            Item* targetTopItem = info->motionItem;
            while(true){
                auto parentItem = targetTopItem->parentItem();
                if(!parentItem || parentItem == currentScenarioItem){
                    break;
                }
                targetTopItem = parentItem;
            }

            ItemList<> items;
            items.extractSubTreeItems(currentScenarioItem);
            for(auto& item : items){
                if(dynamic_pointer_cast<AudioItem>(item) ||
                   dynamic_pointer_cast<MediaItem>(item)){
                    itv->checkItem(item, item->isOwnedBy(targetTopItem));
                }
            }

            if(sequentialCheck.isChecked()){
                info->startButton.setEnabled(false);
            }

            playbackStopConnection.disconnect();
            playbackStopConnection =
                timeBar->sigPlaybackStopped().connect(
                    std::bind(&ScenarioViewImpl::onPlaybackStopped, this, index));

            timeBar->setTime(0.0);
            timeBar->startPlayback();
        }
    }
}


void ScenarioViewImpl::onPlaybackStopped(int index)
{
    playbackStopConnection.disconnect();

    if(sequentialCheck.isChecked()){
        if(index < motions.size()){
            int nextIndex = index + 1;
            if(nextIndex < motions.size()){
                motions[nextIndex]->startButton.setEnabled(true);
            }
        }
    }
}
