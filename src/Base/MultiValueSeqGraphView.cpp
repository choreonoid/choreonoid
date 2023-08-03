#include "MultiValueSeqGraphView.h"
#include "ViewManager.h"
#include "gettext.h"

using namespace std;
using namespace cnoid;


void MultiValueSeqGraphView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<MultiValueSeqGraphView>(
        N_("MultiValueSeqGraphView"), N_("Multi Value Seq"));
}


MultiValueSeqGraphView::MultiValueSeqGraphView()
{
    setDefaultLayoutArea(BottomCenterArea);
}


MultiValueSeqGraphView::~MultiValueSeqGraphView()
{

}


int MultiValueSeqGraphView::currentNumParts(const ItemList<>& items) const
{
    auto frontItem = items.front().get();
    if(auto seqItem = dynamic_cast<MultiValueSeqItem*>(frontItem)){
        return seqItem->seq()->numParts();
    } else if(auto seqItem = dynamic_cast<Vector3SeqItem*>(frontItem)){
        return 3;
    }
    return 0;
}


ItemList<> MultiValueSeqGraphView::extractTargetItems(const ItemList<>& items) const
{
    ItemList<> extracted;
    for(auto& item : items){
        if(auto seqItem = dynamic_cast<MultiValueSeqItem*>(item.get())){
            extracted.push_back(seqItem);
        } else if(auto seqItem = dynamic_cast<Vector3SeqItem*>(item.get())){
            extracted.push_back(seqItem);
        }
    }
    return extracted;
}


void MultiValueSeqGraphView::addGraphDataHandlers(Item* item, int partIndex, std::vector<GraphDataHandlerPtr>& out_handlers)
{
    auto handler = make_shared<GraphDataHandler>();;
    handler->setID(partIndex);
    handler->setLabel(std::to_string(partIndex));
    
    if(auto seqItem = dynamic_cast<MultiValueSeqItem*>(item)){
        auto seq = seqItem->seq();
        if(partIndex < seq->numParts()){
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
    } else if(auto seqItem = dynamic_cast<Vector3SeqItem*>(item)){
        if(partIndex < 3){
            auto seq = seqItem->seq();
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
}


void MultiValueSeqGraphView::updateGraphDataHandler(Item* item, GraphDataHandlerPtr handler)
{
    AbstractSeq* seq = nullptr;
    if(auto seqItem = dynamic_cast<MultiValueSeqItem*>(item)){
        seq = seqItem->seq().get();
    } else if(auto seqItem = dynamic_cast<Vector3SeqItem*>(item)){
        seq = seqItem->seq().get();
    }
    if(seq){
        handler->setFrameProperties(seq->getNumFrames(), seq->getFrameRate());
        handler->update();
    }
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
    auto part = item->seq()->part(partIndex);
    for(int i=0; i < size; ++i){
        part[frame + i] = values[i];
    }
    GraphViewBase::notifyUpdateByEditing(item);
}


// For Vector3Seq
void MultiValueSeqGraphView::onDataRequest
(std::shared_ptr<Vector3Seq> seq, int partIndex, int frame, int size, double* out_values)
{
    if(partIndex < 3){
        for(int i=0; i < size; ++i){
            out_values[i] = seq->frame(i)[partIndex];
        }
    }
}


// For Vector3Seq
void MultiValueSeqGraphView::onDataModified
(Vector3SeqItem* item, int partIndex, int frame, int size, double* values)
{
    if(partIndex < 3){
        auto seq = item->seq();
        for(int i=0; i < size; ++i){
            seq->frame(i)[partIndex] = values[i];
        }
    }
    GraphViewBase::notifyUpdateByEditing(item);
}
