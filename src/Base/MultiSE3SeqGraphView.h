/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_MULTI_SE3_SEQ_GRAPH_VIEW_H
#define CNOID_BASE_MULTI_SE3_SEQ_GRAPH_VIEW_H

#include "GraphViewBase.h"
#include "MultiSE3SeqItem.h"
#include "Buttons.h"
#include <cnoid/ConnectionSet>

namespace cnoid {

class MultiSE3SeqGraphView : public GraphViewBase
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    MultiSE3SeqGraphView();
    ~MultiSE3SeqGraphView();

    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
            
private:
    ToggleToolButton xyzToggles[3];
    ToggleToolButton rpyToggles[3];
    ConnectionSet toggleConnections;

    void setupElementToggleSet(QBoxLayout* box, ToggleToolButton toggles[], const char* labels[], bool isActive);
        
    virtual int currentNumParts(const ItemList<>& items) const;
    virtual ItemList<> extractTargetItems(const ItemList<>& items) const;
    void addGraphDataHandlers(Item* item, int partIndex, std::vector<GraphDataHandlerPtr>& out_handlers);
    void updateGraphDataHandler(Item* item, GraphDataHandlerPtr handler);
    void onDataRequest(std::shared_ptr<MultiSE3Seq> seq, int partIndex, int type, int axis, int frame, int size, double* out_values);
    void onDataModified(MultiSE3SeqItem* item, int partIndex, int type, int axis, int frame, int size, double* values);
};

}

#endif
