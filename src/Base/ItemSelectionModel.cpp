/**
   @author Shin'ichiro Nakaoka
*/

#include "ItemSelectionModel.h"

using namespace cnoid;


ItemSelectionModel::ItemSelectionModel(QAbstractItemModel* model)
    : QItemSelectionModel(model)
{
    initialize();
}


ItemSelectionModel::ItemSelectionModel(QAbstractItemModel* model, QObject* parent)
    : QItemSelectionModel(model, parent)
{
    initialize();
}


void ItemSelectionModel::initialize()
{
    connect(this, SIGNAL(currentChanged(const QModelIndex&, const QModelIndex&)),
            this, SLOT(onCurrentChanged(const QModelIndex&, const QModelIndex&)));

    connect(this, SIGNAL(currentColumnChanged(const QModelIndex&, const QModelIndex&)),
            this, SLOT(onCurrentColumnChanged(const QModelIndex&, const QModelIndex&)));

    connect(this, SIGNAL(currentRowChanged(const QModelIndex&, const QModelIndex&)),
            this, SLOT(onCurrentRowChanged(const QModelIndex&, const QModelIndex&)));

    connect(this, SIGNAL(selectionChanged(const QItemSelection&, const QItemSelection&)),
            this, SLOT(onSelectionChanged(const QItemSelection&, const QItemSelection&)));
}


void ItemSelectionModel::onCurrentChanged(const QModelIndex& index, const QModelIndex& previous)
{
    sigCurrentChanged_(index, previous);
}


void ItemSelectionModel::onCurrentColumnChanged(const QModelIndex& index, const QModelIndex& previous)
{
    sigCurrentColumnChanged_(index, previous);
}


void ItemSelectionModel::onCurrentRowChanged(const QModelIndex& index, const QModelIndex& previous)
{
    sigCurrentRowChanged_(index, previous);
}


void ItemSelectionModel::onSelectionChanged(const QItemSelection& selected, const QItemSelection& deselected)
{
    sigSelectionChanged_(selected, deselected);
}

