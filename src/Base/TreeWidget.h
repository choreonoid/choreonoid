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
    TreeWidget(QWidget* parent = 0);
    virtual ~TreeWidget();

    void setHeaderSectionResizeMode(int column, QHeaderView::ResizeMode mode);
    void setVerticalGridLineShown(bool on);

    SignalProxy<void(QTreeWidgetItem* current, QTreeWidgetItem* previous)> sigCurrentItemChanged() {
        return sigCurrentItemChanged_;
    }
    /*
    SignalProxy<void(QTreeWidgetItem* item, int column)> sigItemActivated() {
        return sigItemActivated_;
    }
    */
    
    SignalProxy<void(QTreeWidgetItem* item, int column)> sigItemActivated() {
        return sigItemActivated_;
    }
    SignalProxy<void(QTreeWidgetItem* item, int column)> sigItemChanged() {
        return sigItemChanged_;
    }
    SignalProxy<void(QTreeWidgetItem* item, int column)> sigItemClicked() {
        return sigItemClicked_;
    }
    SignalProxy<void(QTreeWidgetItem* item)> sigItemCollapsed() {
        return sigItemCollapsed_;
    }
    SignalProxy<void(QTreeWidgetItem* item, int column)> sigItemDoubleClicked() {
        return sigItemDoubleClicked_;
    }
    SignalProxy<void(QTreeWidgetItem* item, int column)> sigItemEntered() {
        return sigItemEntered_;
    }
    SignalProxy<void(QTreeWidgetItem* item)> sigItemExpanded() {
        return sigItemExpanded_;
    }
    SignalProxy<void(QTreeWidgetItem* item, int column)> sigItemPressed() {
        return sigItemPressed_;
    }
    SignalProxy<void()> sigItemSelectionChanged() {
        return sigItemSelectionChanged_;
    }

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

private:
    Signal<void(QTreeWidgetItem* current, QTreeWidgetItem* previous)> sigCurrentItemChanged_;
    Signal<void(QTreeWidgetItem* item, int column)> sigItemActivated_;
    Signal<void(QTreeWidgetItem* item, int column)> sigItemChanged_;
    Signal<void(QTreeWidgetItem* item, int column)> sigItemClicked_;
    Signal<void(QTreeWidgetItem* item)> sigItemCollapsed_;
    Signal<void(QTreeWidgetItem* item, int column)> sigItemDoubleClicked_;
    Signal<void(QTreeWidgetItem* item, int column)> sigItemEntered_;
    Signal<void(QTreeWidgetItem* item)> sigItemExpanded_;
    Signal<void(QTreeWidgetItem* item, int column)> sigItemPressed_;
    Signal<void()> sigItemSelectionChanged_;

    int gridColorRGB;
    bool isVerticalGridLineShown;
};

}

#endif
