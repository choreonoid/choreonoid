/**
   @author Shin'ichiro Nakaoka
*/

#include "MultiValueSeqGraphView.h"
#include "ViewManager.h"
#include "gettext.h"

using namespace std;
using namespace cnoid;


void MultiValueSeqGraphView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<MultiValueSeqGraphView>(
        "MultiValueSeqGraphView", N_("Multi Value Seq"), ViewManager::SINGLE_OPTIONAL);    
}


MultiValueSeqGraphView::MultiValueSeqGraphView()
{
    setDefaultLayoutArea(View::BOTTOM);
}


MultiValueSeqGraphView::~MultiValueSeqGraphView()
{

}


int MultiValueSeqGraphView::currentNumParts(const ItemList<>& items) const
{
    MultiValueSeqItem* item = static_cast<MultiValueSeqItem*>(items.front().get());
    return item->seq()->numParts();
}


ItemList<> MultiValueSeqGraphView::extractTargetItems(const ItemList<>& items) const
{
    return ItemList<MultiValueSeqItem>(items);
}


void MultiValueSeqGraphView::addGraphDataHandlers(Item* item, int partIndex, std::vector<GraphDataHandlerPtr>& out_handlers)
{
    auto seqItem = static_cast<MultiValueSeqItem*>(item);
    auto seq = seqItem->seq();

    if(partIndex < seq->numParts()){

        GraphDataHandlerPtr handler(new GraphDataHandler());
        handler->setID(partIndex);
        handler->setLabel(std::to_string(partIndex));
        //handler->setValueLimits(llimit, ulimit);
        //handler->setVelocityLimits(lvlimit, uvlimit);
        handler->setFrameProperties(seq->numFrames(), seq->frameRate());
        handler->setDataRequestCallback(
            [this, seq, partIndex](int frame, int size, double* out_values){
                onDataRequest(seq, partIndex, frame, size, out_values); });
        handler->setDataModifiedCallback(
            [this, seqItem, partIndex](int frame, int size, double* out_values){
                onDataModified(seqItem, partIndex, frame, size, out_values); });

        out_handlers.push_back(handler);
    }
}


void MultiValueSeqGraphView::updateGraphDataHandler(Item* item, GraphDataHandlerPtr handler)
{
    auto seq = static_cast<MultiValueSeqItem*>(item)->seq();
    handler->setFrameProperties(seq->numFrames(), seq->frameRate());
    handler->update();
}


void MultiValueSeqGraphView::onDataRequest
(std::shared_ptr<MultiValueSeq> seq, int partIndex, int frame, int size, double* out_values)
{
    MultiValueSeq::Part part = seq->part(partIndex);
    for(int i=0; i < size; ++i){
        out_values[i] = part[frame + i];
    }
}


void MultiValueSeqGraphView::onDataModified
(MultiValueSeqItem* item, int partIndex, int frame, int size, double* values)
{
    MultiValueSeq::Part part = item->seq()->part(partIndex);
    for(int i=0; i < size; ++i){
        part[frame + i] = values[i];
    }

    GraphViewBase::notifyUpdateByEditing(item);
}
