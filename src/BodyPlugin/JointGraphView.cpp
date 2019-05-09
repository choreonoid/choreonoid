/**
   @author Shin'ichiro Nakaoka
*/

#include "JointGraphView.h"
#include <cnoid/ItemTreeView>
#include <cnoid/Archive>
#include <cnoid/Link>
#include <cnoid/ViewManager>
#include <QBoxLayout>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;


void JointGraphView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<JointGraphView>(
        "JointGraphView", N_("Joint Trajectories"), ViewManager::SINGLE_OPTIONAL);
}


JointGraphView::JointGraphView()
    : graph(this)
{
    setDefaultLayoutArea(View::BOTTOM);
    
    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->addWidget(&graph);
    setLayout(vbox);

    itemTreeViewConnection = 
        ItemTreeView::mainInstance()->sigSelectionChanged().connect(
            std::bind(&JointGraphView::onItemSelectionChanged, this, _1));

    linkSelection = LinkSelectionView::mainInstance();
}


JointGraphView::~JointGraphView()
{
    itemTreeViewConnection.disconnect();
    bodyItemConnections.disconnect();
}


QWidget* JointGraphView::indicatorOnInfoBar()
{
    return &graph.statusLabel();
}


void JointGraphView::onItemSelectionChanged(const ItemList<MultiValueSeqItem>& items)
{
    if(items.empty()){
        return;
    }

    if(itemInfos.size() == items.size()){
        bool unchanged = true;
        int i=0;
        for(list<ItemInfo>::iterator it = itemInfos.begin(); it != itemInfos.end(); ++it){
            if(it->item != items[i++]){
                unchanged = false;
                break;
            }
        }
        if(unchanged){
            return;
        }
    }
            
    itemInfos.clear();

    for(size_t i=0; i < items.size(); ++i){
        BodyItemPtr bodyItem = items[i]->findOwnerItem<BodyItem>();
        if(bodyItem){
            itemInfos.push_back(ItemInfo());
            list<ItemInfo>::iterator it = --itemInfos.end();
            it->item = items[i];
            it->seq = it->item->seq();
            it->bodyItem = bodyItem;

            it->connections.add(it->item->sigUpdated().connect(
                                    std::bind(&JointGraphView::onDataItemUpdated, this, it)));

            it->connections.add(it->item->sigDetachedFromRoot().connect(
                                    std::bind(&JointGraphView::onDataItemDetachedFromRoot, this, it)));
        }
    }

    updateBodyItems();
    setupGraphWidget();
}


void JointGraphView::onDataItemDetachedFromRoot(std::list<ItemInfo>::iterator itemInfoIter)
{
    itemInfos.erase(itemInfoIter);
    updateBodyItems();
    setupGraphWidget();
}


void JointGraphView::updateBodyItems()
{
    bodyItemConnections.disconnect();
    bodyItems.clear();
    
    for(list<ItemInfo>::iterator it = itemInfos.begin(); it != itemInfos.end(); ++it){

        set<BodyItemPtr>::iterator p = bodyItems.find(it->bodyItem);

        if(p == bodyItems.end()){

            bodyItems.insert(it->bodyItem);

            bodyItemConnections.add(
                linkSelection->sigSelectionChanged(it->bodyItem).connect(
                    std::bind(&JointGraphView::setupGraphWidget, this)));
            
            bodyItemConnections.add(
                it->bodyItem->sigDetachedFromRoot().connect(
                    std::bind(&JointGraphView::onBodyItemDetachedFromRoot, this, it->bodyItem)));
        }
    }
}


void JointGraphView::onBodyItemDetachedFromRoot(BodyItemPtr bodyItem)
{
    bool erased = false;
    list<ItemInfo>::iterator it = itemInfos.begin();
    while(it != itemInfos.end()){
        if(it->bodyItem == bodyItem){
            it = itemInfos.erase(it);
            erased = true;
        } else {
            ++it;
        }
    }
    if(erased){
        updateBodyItems();
        setupGraphWidget();
    }
}


void JointGraphView::setupGraphWidget()
{
    graph.clearDataHandlers();

    for(list<ItemInfo>::iterator it = itemInfos.begin(); it != itemInfos.end(); ++it){

        if(it->bodyItem){

            auto seq = it->item->seq();
            int numParts = seq->numParts();
            auto body = it->bodyItem->body();
            const std::vector<int>& selectedLinkIndices = linkSelection->selectedLinkIndices(it->bodyItem);
            
            for(size_t i=0; i < selectedLinkIndices.size(); ++i){
                Link* link = body->link(selectedLinkIndices[i]);
                if(link && link->jointId() >= 0 && link->jointId() < numParts){
                    addJointTrajectory(it, link, seq);
                }
            }
        }
    }
}


void JointGraphView::addJointTrajectory
(std::list<ItemInfo>::iterator itemInfoIter, Link* joint, std::shared_ptr<MultiValueSeq> seq)
{
    GraphDataHandlerPtr handler(new GraphDataHandler());

    handler->setLabel(joint->name());
    handler->setValueLimits(joint->q_lower(), joint->q_upper());
    handler->setVelocityLimits(joint->dq_lower(), joint->dq_upper());
                
    handler->setFrameProperties(seq->numFrames(), seq->frameRate());
    handler->setDataRequestCallback(
        std::bind(&JointGraphView::onDataRequest, this, itemInfoIter, joint->jointId(), _1, _2, _3));
    handler->setDataModifiedCallback(
        std::bind(&JointGraphView::onDataModified, this, itemInfoIter, joint->jointId(), _1, _2, _3));
                
    graph.addDataHandler(handler);
    itemInfoIter->handlers.push_back(handler);
}


void JointGraphView::onDataItemUpdated(std::list<ItemInfo>::iterator itemInfoIter)
{
    auto seq = itemInfoIter->item->seq();
    int newNumFrames = seq->numFrames();
    double newFrameRate = seq->frameRate();
    
    for(size_t i=0; i < itemInfoIter->handlers.size(); ++i){
        itemInfoIter->handlers[i]->setFrameProperties(newNumFrames, newFrameRate);
        itemInfoIter->handlers[i]->update();
    }
}


void JointGraphView::onDataRequest
(std::list<ItemInfo>::iterator itemInfoIter, int jointId, int frame, int size, double* out_values)
{
    MultiValueSeq::Part part = itemInfoIter->seq->part(jointId);
    for(int i=0; i < size; ++i){
        out_values[i] = part[frame + i];
    }
}


void JointGraphView::onDataModified
(std::list<ItemInfo>::iterator itemInfoIter, int jointId, int frame, int size, double* values)
{
    MultiValueSeq::Part part = itemInfoIter->seq->part(jointId);
    for(int i=0; i < size; ++i){
        part[frame + i] = values[i];
    }
    
    itemInfoIter->connections.block();
    itemInfoIter->item->notifyUpdate();
    itemInfoIter->connections.unblock();
}


bool JointGraphView::storeState(Archive& archive)
{
    return graph.storeState(archive);
}


bool JointGraphView::restoreState(const Archive& archive)
{
    return graph.restoreState(archive);
}
