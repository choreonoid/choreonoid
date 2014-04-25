/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_ITEM_SELECTION_MODEL_H_INCLUDED
#define CNOID_GUIBASE_ITEM_SELECTION_MODEL_H_INCLUDED

#include <cnoid/SignalProxy>
#include <QItemSelectionModel>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ItemSelectionModel : public QItemSelectionModel
{
    Q_OBJECT

        public:
    ItemSelectionModel(QAbstractItemModel* model);
    ItemSelectionModel(QAbstractItemModel* model, QObject* parent);

    inline SignalProxy< boost::signal<void(const QModelIndex& index, const QModelIndex& previous)> > sigCurrentChanged() {
        return sigCurrentChanged_;
    }
    inline SignalProxy< boost::signal<void(const QModelIndex& index, const QModelIndex& previous)> > sigCurrentColumnChanged() {
        return sigCurrentColumnChanged_;
    }
    inline SignalProxy< boost::signal<void(const QModelIndex& index, const QModelIndex& previous)> > sigCurrentRowChanged() {
        return sigCurrentRowChanged_;
    }
    inline SignalProxy< boost::signal<void(const QItemSelection& selected, const QItemSelection& deselected)> > sigSelectionChanged() {
        return sigSelectionChanged_;
    }

private Q_SLOTS:
    void onCurrentChanged(const QModelIndex& index, const QModelIndex& previous);
    void onCurrentColumnChanged(const QModelIndex& index, const QModelIndex& previous);
    void onCurrentRowChanged(const QModelIndex& index, const QModelIndex& previous);
    void onSelectionChanged(const QItemSelection& selected, const QItemSelection& deselected);

private:
    boost::signal<void(const QModelIndex& index, const QModelIndex& previous)> sigCurrentChanged_;
    boost::signal<void(const QModelIndex& index, const QModelIndex& previous)> sigCurrentColumnChanged_;
    boost::signal<void(const QModelIndex& index, const QModelIndex& previous)> sigCurrentRowChanged_;
    boost::signal<void(const QItemSelection& selected, const QItemSelection& deselected)> sigSelectionChanged_;

    void initialize();
};
}

#endif

