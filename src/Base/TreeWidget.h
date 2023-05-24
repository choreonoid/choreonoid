#ifndef CNOID_BASE_TREE_WIDGET_H
#define CNOID_BASE_TREE_WIDGET_H

#include <cnoid/Signal>
#include <QTreeWidget>
#include <QHeaderView>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT TreeWidget : public QTreeWidget
{
public:
    TreeWidget(QWidget* parent = nullptr);
    ~TreeWidget();

    void setHeaderSectionResizeMode(int column, QHeaderView::ResizeMode mode);
    void setVerticalGridLineShown(bool on);

    void setUserInputEnabled(bool on) { isUserInputEnabled_ = on; }
    bool isUserInputEnabled() const { return isUserInputEnabled_; }

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
    SignalProxy<void(const QModelIndex& parent, int first, int last)> sigRowsAboutToBeRemoved();
    SignalProxy<void(const QModelIndex& parent, int first, int last)> sigRowsRemoved();
    SignalProxy<void(const QModelIndex& parent, int first, int last)> sigRowsInserted();

    // Signal of QHeaderView owned by TreeWidget
    SignalProxy<void(int logicalIndex, int oldSize, int newSize)> sigSectionResized();

    // Signal of QWidget
    SignalProxy<void(const QPoint& pos)> sigCustomContextMenuRequested();

protected:
    virtual void paintEvent(QPaintEvent* event) override;
    virtual void scrollContentsBy(int dx, int dy) override;
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    virtual void wheelEvent(QWheelEvent* event) override;

private:
    class Impl;
    Impl* impl;
    bool isUserInputEnabled_;
};

}

#endif
