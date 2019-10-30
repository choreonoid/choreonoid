/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_TREE_WIDGET_H
#define CNOID_BASE_TREE_WIDGET_H

#include <cnoid/Signal>
#include <QTreeWidget>
#include <QHeaderView>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT TreeWidget : public QTreeWidget
{
    Q_OBJECT

public:
    TreeWidget(QWidget* parent = nullptr);
    ~TreeWidget();

    void setHeaderSectionResizeMode(int column, QHeaderView::ResizeMode mode);
    void setVerticalGridLineShown(bool on);

    SignalProxy<void(QTreeWidgetItem* current, QTreeWidgetItem* previous)> sigCurrentItemChanged();
    SignalProxy<void(QTreeWidgetItem* item, int column)> sigItemActivated();
    SignalProxy<void(QTreeWidgetItem* item, int column)> sigItemChanged();
    SignalProxy<void(QTreeWidgetItem* item, int column)> sigItemClicked();
    SignalProxy<void(QTreeWidgetItem* item)> sigItemCollapsed();
    SignalProxy<void(QTreeWidgetItem* item, int column)> sigItemDoubleClicked();
    SignalProxy<void(QTreeWidgetItem* item, int column)> sigItemEntered();
    SignalProxy<void(QTreeWidgetItem* item)> sigItemExpanded();
    SignalProxy<void(QTreeWidgetItem* item, int column)> sigItemPressed();
    SignalProxy<void()> sigItemSelectionChanged();

    // Signals of QAbstractItemModel owned by TreeWidget
    SignalProxy<void(const QModelIndex &parent, int first, int last)> sigRowsAboutToBeRemoved();
    SignalProxy<void(const QModelIndex &parent, int first, int last)> sigRowsRemoved();
    SignalProxy<void(const QModelIndex &parent, int first, int last)> sigRowsInserted();

    // Signal of QHeaderView owned by TreeWidget
    SignalProxy<void(int logicalIndex, int oldSize, int newSize)> sigSectionResized();

protected:
    virtual void paintEvent(QPaintEvent* event);
    virtual void scrollContentsBy(int dx, int dy);     

private Q_SLOTS:
    void onCurrentItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous);
    void onItemActivated(QTreeWidgetItem* item, int column);
    void onItemChanged(QTreeWidgetItem* item, int column);
    void onItemClicked(QTreeWidgetItem* item, int column);
    void onItemCollapsed(QTreeWidgetItem* item);
    void onItemDoubleClicked(QTreeWidgetItem* item, int column);
    void onItemEntered(QTreeWidgetItem* item, int column);
    void onItemExpanded(QTreeWidgetItem* item);
    void onItemPressed(QTreeWidgetItem* item, int column);
    void onItemSelectionChanged(void);
    void onRowsAboutToBeRemoved(const QModelIndex &parent, int first, int last);
    void onRowsRemoved(const QModelIndex &parent, int first, int last);
    void onRowsInserted(const QModelIndex &parent, int first, int last);
    void onSectionResized(int logicalIndex, int oldSize, int newSize);

private:
    class Impl;
    Impl* impl;
};

}

#endif
