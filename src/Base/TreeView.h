/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_TREE_VIEW_H
#define CNOID_BASE_TREE_VIEW_H

#include "ItemSelectionModel.h"
#include <cnoid/Signal>
#include <QTreeView>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT TreeView : public QTreeView
{
    Q_OBJECT

        public:
    TreeView(QWidget* parent = 0);

    virtual void setModel(QAbstractItemModel* model);

    ItemSelectionModel* itemSelectionModel() const;
        
    SignalProxy<void(const QModelIndex& index)> sigCollapsed() {
        return sigCollapsed_;
    }
    SignalProxy<void(const QModelIndex& index)> sigExpanded() {
        return sigExpanded_;
    }
    SignalProxy<void(const QModelIndex& index)> sigActivated() {
        return sigActivated_;
    }
    SignalProxy<void(const QModelIndex& index)> sigClicked() {
        return sigClicked_;
    }
    SignalProxy<void(const QModelIndex& index)> sigDoubleClicked() {
        return sigDoubleClicked_;
    }
    SignalProxy<void(const QModelIndex& index)> sigEntered() {
        return sigEntered_;
    }
    SignalProxy<void(const QModelIndex& index)> sigPressed() {
        return sigPressed_;
    }
    SignalProxy<void()> sigViewportEntered() {
        return sigViewportEntered_;
    }

private Q_SLOTS:
    void onCollapsed(const QModelIndex& index);
    void onExpanded(const QModelIndex& index);
    void onActivated(const QModelIndex& index);
    void onClicked(const QModelIndex& index);
    void onDoubleClicked(const QModelIndex& index);
    void onEntered(const QModelIndex& index);
    void onPressed(const QModelIndex& index);
    void onViewportEntered(void);

private:
    Signal<void(const QModelIndex& index)> sigCollapsed_;
    Signal<void(const QModelIndex& index)> sigExpanded_;
    Signal<void(const QModelIndex& index)> sigActivated_;
    Signal<void(const QModelIndex& index)> sigClicked_;
    Signal<void(const QModelIndex& index)> sigDoubleClicked_;
    Signal<void(const QModelIndex& index)> sigEntered_;
    Signal<void(const QModelIndex& index)> sigPressed_;
    Signal<void()> sigViewportEntered_;
};

}

#endif
