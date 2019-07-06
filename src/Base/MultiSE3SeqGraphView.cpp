/**
   @author Shin'ichiro Nakaoka
*/

#include "MultiSE3SeqGraphView.h"
#include "ViewManager.h"
#include <cnoid/EigenUtil>
#include "gettext.h"

using namespace std;
using namespace cnoid;


void MultiSE3SeqGraphView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<MultiSE3SeqGraphView>(
        "MultiSE3SeqGraphView", N_("Multi SE3 Seq"), ViewManager::SINGLE_OPTIONAL);
}


MultiSE3SeqGraphView::MultiSE3SeqGraphView()
{
    setDefaultLayoutArea(View::BOTTOM);
    setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    static const char* xyzLabels[] = { "X", "Y", "Z" };
    static const char* rpyLabels[] = { "R", "P", "Y" };

    QVBoxLayout* vbox = new QVBoxLayout();;
    vbox->setSpacing(0);
    setupElementToggleSet(vbox, xyzToggles, xyzLabels, true);
    setupElementToggleSet(vbox, rpyToggles, rpyLabels, false);

    leftVBox()->insertLayout(0, vbox);
}


void MultiSE3SeqGraphView::setupElementToggleSet
(QBoxLayout* box, ToggleToolButton toggles[], const char* labels[], bool isActive)
{
    for(int i=0; i < 3; ++i){
        toggles[i].setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        box->addWidget(&toggles[i]);
        toggles[i].setChecked(isActive);
        toggles[i].setText(labels[i]);

        toggleConnections.add(
            toggles[i].sigToggled().connect(
                [&](bool){ updateSelections(); }));
    }
}


MultiSE3SeqGraphView::~MultiSE3SeqGraphView()
{

}


int MultiSE3SeqGraphView::currentNumParts(const ItemList<>& items) const
{
    MultiSE3SeqItem* item = static_cast<MultiSE3SeqItem*>(items.front().get());
    return item->seq()->numParts();
}


ItemList<> MultiSE3SeqGraphView::extractTargetItems(const ItemList<>& items) const
{
    return ItemList<MultiSE3SeqItem>(items);
}


void MultiSE3SeqGraphView::addGraphDataHandlers(Item* item, int partIndex, std::vector<GraphDataHandlerPtr>& out_handlers)
{
    MultiSE3SeqItem* seqItem = static_cast<MultiSE3SeqItem*>(item);
    auto seq = seqItem->seq();
    
    if(partIndex < seq->numParts()){
    
        for(int i=0; i < 2; ++i){
            ToggleToolButton* toggles = (i == 0) ? xyzToggles : rpyToggles;
            for(int j=0; j < 3; ++j){

                if(toggles[j].isChecked()){
    
                    GraphDataHandlerPtr handler(new GraphDataHandler());

                    handler->setLabel(std::to_string(partIndex));
                    handler->setFrameProperties(seq->numFrames(), seq->frameRate());
                    handler->setDataRequestCallback(
                        [this, seq, partIndex, i, j](int frame, int size, double* out_values){
                            onDataRequest(seq, partIndex, i, j, frame, size, out_values); });
                    handler->setDataModifiedCallback(
                        [this, seqItem, partIndex, i, j](int frame, int size, double* out_values){
                            onDataModified(seqItem, partIndex, i, j, frame, size, out_values); });

                    out_handlers.push_back(handler);
                }
            }
        }
    }
}


void MultiSE3SeqGraphView::updateGraphDataHandler(Item* item, GraphDataHandlerPtr handler)
{
    auto seq = static_cast<MultiSE3SeqItem*>(item)->seq();
    handler->setFrameProperties(seq->numFrames(), seq->frameRate());
    handler->update();
}


void MultiSE3SeqGraphView::onDataRequest
(std::shared_ptr<MultiSE3Seq> seq, int partIndex, int type, int axis, int frame, int size, double* out_values)
{
    MultiSE3Seq::Part part = seq->part(partIndex);
    if(type == 0){ // xyz
        for(int i=0; i < size; ++i){
            out_values[i] = part[frame + i].translation()[axis];
        }
    } else { // rpy
        //! \todo eliminate the redundant calculations when more than one axes are plot at the same time
        for(int i=0; i < size; ++i){
            const SE3& p = part[frame + i];
            out_values[i] = rpyFromRot(Matrix3(p.rotation()))[axis];
        }
    }
}


void MultiSE3SeqGraphView::onDataModified
(MultiSE3SeqItem* item, int partIndex, int type, int axis, int frame, int size, double* values)
{
    MultiSE3Seq::Part part = item->seq()->part(partIndex);
    if(type == 0){ // xyz
        for(int i=0; i < size; ++i){
            SE3& p = part[frame + i];
            p.translation()[axis] = values[i];
        }
    } else { // rpy
        //! \todo make the following code more efficient
        for(int i=0; i < size; ++i){
            SE3& p = part[frame + i];
            Vector3 rpy(rpyFromRot(Matrix3(p.rotation())));
            rpy[axis] = values[i];
            p.rotation() = rotFromRpy(rpy);
        }
    }

    GraphViewBase::notifyUpdateByEditing(item);
}


bool MultiSE3SeqGraphView::storeState(Archive& archive)
{
    if(GraphViewBase::storeState(archive)){
        Listing& visibleElements = *archive.createFlowStyleListing("visibleElements");
        for(int i=0; i < 3; ++i){
            if(xyzToggles[i].isChecked()){
                visibleElements.append(i);
            }
        }
        for(int i=0; i < 3; ++i){
            if(rpyToggles[i].isChecked()){
                visibleElements.append(i+3);
            }
        }
        return true;
    }
    return false;
}


bool MultiSE3SeqGraphView::restoreState(const Archive& archive)
{
    if(GraphViewBase::restoreState(archive)){
        const Listing& visibleElements = *archive.findListing("visibleElements");
        if(visibleElements.isValid()){
            toggleConnections.block();
            for(int i=0; i < 3; ++i){
                xyzToggles[i].setChecked(false);
                rpyToggles[i].setChecked(false);
            }
            for(int i=0; i < visibleElements.size(); ++i){
                int index = visibleElements[i].toInt();
                if(index < 3){
                    xyzToggles[index].setChecked(true);
                } else {
                    rpyToggles[index-3].setChecked(true);
                }
            }
            toggleConnections.unblock();
        }
        return true;
    }
    return false;
}
