/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_ITEM_SELECTION_MODEL_H
#define CNOID_BASE_ITEM_SELECTION_MODEL_H

#include <cnoid/Signal>
#include <QItemSelectionModel>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ItemSelectionModel : public QItemSelectionModel
{
    Q_OBJECT

public:
    ItemSelectionModel(QAbstractItemModel* model);
    ItemSelectionModel(QAbstractItemModel* model, QObject* parent);

    SignalProxy<void(const QModelIndex& index, const QModelIndex& previous)> sigCurrentChanged() {
        return sigCurrentChanged_;
    }
    SignalProxy<void(const QModelIndex& index, const QModelIndex& previous)> sigCurrentColumnChanged() {
        return sigCurrentColumnChanged_;
    }
    SignalProxy<void(const QModelIndex& index, const QModelIndex& previous)> sigCurrentRowChanged() {
        return sigCurrentRowChanged_;
    }
    SignalProxy<void(const QItemSelection& selected, const QItemSelection& deselected)> sigSelectionChanged() {
        return sigSelectionChanged_;
    }

private Q_SLOTS:
    void onCurrentChanged(const QModelIndex& index, const QModelIndex& previous);
    void onCurrentColumnChanged(const QModelIndex& index, const QModelIndex& previous);
    void onCurrentRowChanged(const QModelIndex& index, const QModelIndex& previous);
    void onSelectionChanged(const QItemSelection& selected, const QItemSelection& deselected);

private:
    Signal<void(const QModelIndex& index, const QModelIndex& previous)> sigCurrentChanged_;
    Signal<void(const QModelIndex& index, const QModelIndex& previous)> sigCurrentColumnChanged_;
    Signal<void(const QModelIndex& index, const QModelIndex& previous)> sigCurrentRowChanged_;
    Signal<void(const QItemSelection& selected, const QItemSelection& deselected)> sigSelectionChanged_;

    void initialize();
};

}

#endif

