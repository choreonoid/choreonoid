/**
   @author Shin'ichiro Nakaoka
*/

#include "TreeView.h"
#include "ItemSelectionModel.h"

using namespace cnoid;

TreeView::TreeView(QWidget* parent)
    : QTreeView(parent)
{
    connect(this, SIGNAL(collapsed(const QModelIndex&)),
            this, SLOT(onCollapsed(const QModelIndex&)));
    
    connect(this, SIGNAL(expanded(const QModelIndex&)),
            this, SLOT(onExpanded(const QModelIndex&)));

    connect(this, SIGNAL(activated(const QModelIndex&)),
            this, SLOT(onActivated(const QModelIndex&)));

    connect(this, SIGNAL(clicked(const QModelIndex&)),
            this, SLOT(onClicked(const QModelIndex&)));

    connect(this, SIGNAL(doubleClicked(const QModelIndex&)),
            this, SLOT(onDoubleClicked(const QModelIndex&)));

    connect(this, SIGNAL(entered(const QModelIndex&)),
            this, SLOT(onEntered(const QModelIndex&)));

    connect(this, SIGNAL(pressed(const QModelIndex&)),
            this, SLOT(onPressed(const QModelIndex&)));

    connect(this, SIGNAL(viewportEntered()), this, SLOT(onViewportEntered()));
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


ItemSelectionModel* TreeView::itemSelectionModel() const
{
    return dynamic_cast<ItemSelectionModel*>(selectionModel());
}


void TreeView::onCollapsed(const QModelIndex& index)
{
    sigCollapsed_(index);
}


void TreeView::onExpanded(const QModelIndex& index)
{
    sigExpanded_(index);
}


void TreeView::onActivated(const QModelIndex& index)
{
    sigActivated_(index);
}


void TreeView::onClicked(const QModelIndex& index)
{
    sigClicked_(index);
}


void TreeView::onDoubleClicked(const QModelIndex& index)
{
    sigDoubleClicked_(index);
}


void TreeView::onEntered(const QModelIndex& index)
{
    sigEntered_(index);
}


void TreeView::onPressed(const QModelIndex& index)
{
    sigPressed_(index);
}


void TreeView::onViewportEntered(void)
{
    sigViewportEntered_();
}


