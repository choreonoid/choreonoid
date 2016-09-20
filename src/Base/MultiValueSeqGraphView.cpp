/**
   @author Shin'ichiro Nakaoka
*/

#include "MultiValueSeqGraphView.h"
#include "ViewManager.h"
#include <boost/lexical_cast.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace std::placeholders;


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
    MultiValueSeqItem* seqItem = static_cast<MultiValueSeqItem*>(item);
    MultiValueSeqPtr seq = seqItem->seq();

    if(partIndex < seq->numParts()){

        GraphDataHandlerPtr handler(new GraphDataHandler());
        handler->setID(partIndex);
        handler->setLabel(boost::lexical_cast<string>(partIndex));
        //handler->setValueLimits(llimit, ulimit);
        //handler->setVelocityLimits(lvlimit, uvlimit);
        handler->setFrameProperties(seq->numFrames(), seq->frameRate());
        handler->setDataRequestCallback(
            std::bind(&MultiValueSeqGraphView::onDataRequest, this, seq, partIndex, _1, _2, _3));
        handler->setDataModifiedCallback(
            std::bind(&MultiValueSeqGraphView::onDataModified, this, seqItem, partIndex, _1, _2, _3));

        out_handlers.push_back(handler);
    }
}


void MultiValueSeqGraphView::updateGraphDataHandler(Item* item, GraphDataHandlerPtr handler)
{
    MultiValueSeqPtr seq = static_cast<MultiValueSeqItem*>(item)->seq();
    handler->setFrameProperties(seq->numFrames(), seq->frameRate());
    handler->update();
}


void MultiValueSeqGraphView::onDataRequest
(MultiValueSeqPtr seq, int partIndex, int frame, int size, double* out_values)
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
