/**
   @author Shin'ichiro Nakaoka
*/

#include "TreeView.h"
#include "ItemSelectionModel.h"
#include <cnoid/stdx/optional>

using namespace std;
using namespace cnoid;

namespace cnoid {

class TreeView::Impl
{
public:
    stdx::optional<Signal<void(const QModelIndex& index)>> sigCollapsed;
    stdx::optional<Signal<void(const QModelIndex& index)>> sigExpanded;
    stdx::optional<Signal<void(const QModelIndex& index)>> sigActivated;
    stdx::optional<Signal<void(const QModelIndex& index)>> sigClicked;
    stdx::optional<Signal<void(const QModelIndex& index)>> sigDoubleClicked;
    stdx::optional<Signal<void(const QModelIndex& index)>> sigEntered;
    stdx::optional<Signal<void(const QModelIndex& index)>> sigPressed;
    stdx::optional<Signal<void()>> sigViewportEntered;
};

}


TreeView::TreeView(QWidget* parent)
    : QTreeView(parent)
{
    impl = new Impl;
}


TreeView::~TreeView()
{
    delete impl;
}


void TreeView::setModel(QAbstractItemModel* model)
{
    QItemSelectionModel* old = selectionModel();
    QTreeView::setModel(model);
    if(old && !old->parent()){
        delete old;
    }
    setSelectionModel(new ItemSelectionModel(model, this));
}


ItemSelectionModel* TreeView::selectionModel() const
{
    return dynamic_cast<ItemSelectionModel*>(selectionModel());
}


SignalProxy<void(const QModelIndex& index)> TreeView::sigCollapsed()
{
    if(!impl->sigCollapsed){
        stdx::emplace(impl->sigCollapsed);
        connect(this, SIGNAL(collapsed(const QModelIndex&)),
                this, SLOT(onCollapsed(const QModelIndex&)));
    }
    return *impl->sigCollapsed;
}


SignalProxy<void(const QModelIndex& index)> TreeView::sigExpanded()
{
    if(!impl->sigExpanded){
        stdx::emplace(impl->sigExpanded);
        connect(this, SIGNAL(expanded(const QModelIndex&)),
                this, SLOT(onExpanded(const QModelIndex&)));
    }
    return *impl->sigExpanded;
}


SignalProxy<void(const QModelIndex& index)> TreeView::sigActivated()
{
    if(!impl->sigActivated){
        stdx::emplace(impl->sigActivated);
        connect(this, SIGNAL(activated(const QModelIndex& index)),
                this, SLOT(onActivated(const QModelIndex& index)));
    }
    return *impl->sigActivated;
}


SignalProxy<void(const QModelIndex& index)> TreeView::sigClicked()
{
    if(!impl->sigClicked){
        stdx::emplace(impl->sigClicked);
        connect(this, SIGNAL(clicked(const QModelIndex& index)),
                this, SLOT(onClicked(const QModelIndex& index)));
    }
    return *impl->sigClicked;
}

    
SignalProxy<void(const QModelIndex& index)> TreeView::sigDoubleClicked()
{
    if(!impl->sigDoubleClicked){
        stdx::emplace(impl->sigDoubleClicked);
        connect(this, SIGNAL(doubleClicked(const QModelIndex& index)),
                this, SLOT(onDoubleClicked(const QModelIndex& index)));
    }
    return *impl->sigDoubleClicked;
}
    

SignalProxy<void(const QModelIndex& index)> TreeView::sigEntered()
{
    if(!impl->sigEntered){
        stdx::emplace(impl->sigEntered);
        connect(this, SIGNAL(entered(const QModelIndex& index)),
                this, SLOT(onEntered(const QModelIndex& index)));
    }
    return *impl->sigEntered;
}


SignalProxy<void(const QModelIndex& index)> TreeView::sigPressed()
{
    if(!impl->sigPressed){
        stdx::emplace(impl->sigPressed);
        connect(this, SIGNAL(pressed(const QModelIndex& index)),
                this, SLOT(onPressed(const QModelIndex& index)));
    }
    return *impl->sigPressed;
}


SignalProxy<void()> TreeView::sigViewportEntered()
{
    if(!impl->sigViewportEntered){
        stdx::emplace(impl->sigViewportEntered);
        connect(this, SIGNAL(viewportEntere()),
                this, SLOT(onViewportEntered()));
    }
    return *impl->sigViewportEntered;
}


void TreeView::onCollapsed(const QModelIndex& index)
{
    (*impl->sigCollapsed)(index);
}


void TreeView::onExpanded(const QModelIndex& index)
{
    (*impl->sigExpanded)(index);
}


void TreeView::onActivated(const QModelIndex& index)
{
    (*impl->sigActivated)(index);
}


void TreeView::onClicked(const QModelIndex& index)
{
    (*impl->sigClicked)(index);
}


void TreeView::onDoubleClicked(const QModelIndex& index)
{
    (*impl->sigDoubleClicked)(index);
}


void TreeView::onEntered(const QModelIndex& index)
{
    (*impl->sigEntered)(index);
}


void TreeView::onPressed(const QModelIndex& index)
{
    (*impl->sigPressed)(index);
}


void TreeView::onViewportEntered()
{
    (*impl->sigViewportEntered)();
}
