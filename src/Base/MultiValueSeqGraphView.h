/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_MULTI_VALUE_SEQ_GRAPH_VIEW_H_INCLUDED
#define CNOID_BASE_MULTI_VALUE_SEQ_GRAPH_VIEW_H_INCLUDED

#include "GraphViewBase.h"
#include "MultiValueSeqItem.h"

namespace cnoid {

class MultiValueSeqGraphView : public GraphViewBase
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    MultiValueSeqGraphView();
    ~MultiValueSeqGraphView();
            
private:
    virtual int currentNumParts(const ItemList<>& items) const;
    virtual ItemList<Item> extractTargetItems(const ItemList<>& items) const;
    void addGraphDataHandlers(Item* item, int partIndex, std::vector<GraphDataHandlerPtr>& out_handlers);
    void updateGraphDataHandler(Item* item, GraphDataHandlerPtr handler);
    void onDataRequest(MultiValueSeqPtr seq, int partIndex, int frame, int size, double* out_values);
    void onDataModified(MultiValueSeqItem* item, int partIndex, int frame, int size, double* values);
};
}

#endif
