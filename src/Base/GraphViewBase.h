/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_GRAPH_VIEW_BASE_H
#define CNOID_BASE_GRAPH_VIEW_BASE_H

#include "GraphWidget.h"
#include <cnoid/View>
#include <cnoid/ItemList>
#include <QBoxLayout>

namespace cnoid {

class GraphViewBaseImpl;

class GraphViewBase : public View
{
public:
    GraphViewBase();
    ~GraphViewBase();

    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);

    virtual QWidget* indicatorOnInfoBar();

protected:

    virtual int currentNumParts(const ItemList<>& items) const;
    virtual ItemList<> extractTargetItems(const ItemList<>& items) const = 0;
    virtual void addGraphDataHandlers(Item* item, int partIndex, std::vector<GraphDataHandlerPtr>& out_handlers) = 0;
    virtual void updateGraphDataHandler(Item* item, GraphDataHandlerPtr handler) = 0;

    void updateSelections();
    void notifyUpdateByEditing(Item* item);

    QVBoxLayout* leftVBox() const;

private:
    GraphViewBaseImpl* impl;
    friend class GraphViewBaseImpl;
};

}

#endif
