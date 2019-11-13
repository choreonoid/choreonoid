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
    ~TreeView();

    virtual void setModel(QAbstractItemModel* model);

    ItemSelectionModel* selectionModel() const;
        
    SignalProxy<void(const QModelIndex& index)> sigCollapsed();
    SignalProxy<void(const QModelIndex& index)> sigExpanded();
    SignalProxy<void(const QModelIndex& index)> sigActivated();
    SignalProxy<void(const QModelIndex& index)> sigClicked();
    SignalProxy<void(const QModelIndex& index)>	sigDoubleClicked();
    SignalProxy<void(const QModelIndex& index)>	sigEntered();
    SignalProxy<void(const QModelIndex& index)>	sigPressed();
    SignalProxy<void()> sigViewportEntered();


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

private:
    class Impl;
    Impl* impl;
};

}

#endif
