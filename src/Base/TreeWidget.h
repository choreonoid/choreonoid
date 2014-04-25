/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_TREE_WIDGET_H_INCLUDED
#define CNOID_GUIBASE_TREE_WIDGET_H_INCLUDED

#include <cnoid/SignalProxy>
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

    inline SignalProxy< boost::signal<void(QTreeWidgetItem* current, QTreeWidgetItem* previous)> > sigCurrentItemChanged() {
        return sigCurrentItemChanged_;
    }
    inline SignalProxy< boost::signal<void(QTreeWidgetItem* item, int column)> > sigItemActivated() {
        return sigItemActivated_;
    }
    inline SignalProxy< boost::signal<void(QTreeWidgetItem* item, int column)> > sigItemChanged() {
        return sigItemChanged_;
    }
    inline SignalProxy< boost::signal<void(QTreeWidgetItem* item, int column)> > sigItemClicked() {
        return sigItemClicked_;
    }
    inline SignalProxy< boost::signal<void(QTreeWidgetItem* item)> > sigItemCollapsed() {
        return sigItemCollapsed_;
    }
    inline SignalProxy< boost::signal<void(QTreeWidgetItem* item, int column)> > sigItemDoubleClicked() {
        return sigItemDoubleClicked_;
    }
    inline SignalProxy< boost::signal<void(QTreeWidgetItem* item, int column)> > sigItemEntered() {
        return sigItemEntered_;
    }
    inline SignalProxy< boost::signal<void(QTreeWidgetItem* item)> > sigItemExpanded() {
        return sigItemExpanded_;
    }
    inline SignalProxy< boost::signal<void(QTreeWidgetItem* item, int column)> > sigItemPressed() {
        return sigItemPressed_;
    }
    inline SignalProxy< boost::signal<void()> > sigItemSelectionChanged() {
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
    boost::signal<void(QTreeWidgetItem* current, QTreeWidgetItem* previous)> sigCurrentItemChanged_;
    boost::signal<void(QTreeWidgetItem* item, int column)> sigItemActivated_;
    boost::signal<void(QTreeWidgetItem* item, int column)> sigItemChanged_;
    boost::signal<void(QTreeWidgetItem* item, int column)> sigItemClicked_;
    boost::signal<void(QTreeWidgetItem* item)> sigItemCollapsed_;
    boost::signal<void(QTreeWidgetItem* item, int column)> sigItemDoubleClicked_;
    boost::signal<void(QTreeWidgetItem* item, int column)> sigItemEntered_;
    boost::signal<void(QTreeWidgetItem* item)> sigItemExpanded_;
    boost::signal<void(QTreeWidgetItem* item, int column)> sigItemPressed_;
    boost::signal<void()> sigItemSelectionChanged_;

    int gridColorRGB;
    bool isVerticalGridLineShown;
};
}

#endif
