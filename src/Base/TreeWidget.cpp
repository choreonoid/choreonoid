/**
   @author Shin'ichiro Nakaoka
*/

#include "TreeWidget.h"
#include <QHeaderView>
#include <QPainter>
#include <QPen>
#include <cnoid/stdx/optional>

using namespace cnoid;

namespace cnoid {

class TreeWidget::Impl
{
public:
    int gridColorRGB;
    bool isVerticalGridLineShown;
    
    stdx::optional<Signal<void(QTreeWidgetItem* current, QTreeWidgetItem* previous)>> sigCurrentItemChanged;
    stdx::optional<Signal<void(QTreeWidgetItem* item, int column)>> sigItemActivated;
    stdx::optional<Signal<void(QTreeWidgetItem* item, int column)>> sigItemChanged;
    stdx::optional<Signal<void(QTreeWidgetItem* item, int column)>> sigItemClicked;
    stdx::optional<Signal<void(QTreeWidgetItem* item)>> sigItemCollapsed;
    stdx::optional<Signal<void(QTreeWidgetItem* item, int column)>> sigItemDoubleClicked;
    stdx::optional<Signal<void(QTreeWidgetItem* item, int column)>> sigItemEntered;
    stdx::optional<Signal<void(QTreeWidgetItem* item)>> sigItemExpanded;
    stdx::optional<Signal<void(QTreeWidgetItem* item, int column)>> sigItemPressed;
    stdx::optional<Signal<void()>> sigItemSelectionChanged;

    stdx::optional<Signal<void(const QModelIndex& parent, int first, int last)>> sigRowsAboutToBeRemoved;
    stdx::optional<Signal<void(const QModelIndex& parent, int first, int last)>> sigRowsRemoved;
    stdx::optional<Signal<void(const QModelIndex& parent, int first, int last)>> sigRowsInserted;

    stdx::optional<Signal<void(int logicalIndex, int oldSize, int newSize)>> sigSectionResized;

    Impl();
};

}


TreeWidget::TreeWidget(QWidget* parent)
    : QTreeWidget(parent)
{
    impl = new Impl;
}


TreeWidget::Impl::Impl()
{
    isVerticalGridLineShown = false;
}


TreeWidget::~TreeWidget()
{
    delete impl;
}


void TreeWidget::setHeaderSectionResizeMode(int column, QHeaderView::ResizeMode mode)
{
    header()->setSectionResizeMode(column, mode);
}
    

void TreeWidget::setVerticalGridLineShown(bool on)
{
    if(on){
        QStyleOptionViewItem option;
        impl->gridColorRGB = style()->styleHint(QStyle::SH_Table_GridLineColor, &option);
    }
    impl->isVerticalGridLineShown = on;
}
        

void TreeWidget::paintEvent(QPaintEvent* event)
{
    QTreeWidget::paintEvent(event);

    if(impl->isVerticalGridLineShown && topLevelItemCount()){
        QHeaderView* hv = header();
        QPainter painter(viewport());
        QPen oldPen = painter.pen();
        painter.setPen(QPen(QColor::fromRgb(impl->gridColorRGB)));
    
        for(int i = 0; i < hv->count(); ++i){
            // draw only visible sections starting from second column
            if(hv->isSectionHidden(i) || hv->visualIndex(i) <= 0){
                continue;
            }
            // position mapped to viewport
            int pos = hv->sectionViewportPosition(i) - 1;
            if(pos > 0){
                painter.drawLine(QPoint(pos, 0), QPoint(pos, height()));
            }
        }
        painter.setPen(oldPen);
    }
}

 
void TreeWidget::scrollContentsBy(int dx, int dy)
{
    QTreeWidget::scrollContentsBy(dx, dy);
    // make sure lines get updated even if the view is empty
    viewport()->update();
}


SignalProxy<void(QTreeWidgetItem* current, QTreeWidgetItem* previous)> TreeWidget::sigCurrentItemChanged()
{
    if(!impl->sigCurrentItemChanged){
        stdx::emplace(impl->sigCurrentItemChanged);
        connect(this, SIGNAL(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)),
            this, SLOT(onCurrentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)));
    }
    return *impl->sigCurrentItemChanged;
}


SignalProxy<void(QTreeWidgetItem* item, int column)> TreeWidget::sigItemActivated()
{
    if(!impl->sigItemActivated){
        stdx::emplace(impl->sigItemActivated);
        connect(this, SIGNAL(itemActivated(QTreeWidgetItem*, int)),
                this, SLOT(onItemActivated(QTreeWidgetItem*, int)));
    }
    return *impl->sigItemActivated;
}


SignalProxy<void(QTreeWidgetItem* item, int column)> TreeWidget::sigItemChanged()
{
    if(!impl->sigItemChanged){
        stdx::emplace(impl->sigItemChanged);
        connect(this, SIGNAL(itemChanged(QTreeWidgetItem*, int)),
                this, SLOT(onItemChanged(QTreeWidgetItem*, int)));
    }
    return *impl->sigItemChanged;
}


SignalProxy<void(QTreeWidgetItem* item, int column)> TreeWidget::sigItemClicked()
{
    if(!impl->sigItemClicked){
        stdx::emplace(impl->sigItemClicked);
        connect(this, SIGNAL(itemClicked(QTreeWidgetItem*, int)),
                this, SLOT(onItemClicked(QTreeWidgetItem*, int)));
    }
    return *impl->sigItemClicked;
}


SignalProxy<void(QTreeWidgetItem* item)> TreeWidget::sigItemCollapsed()
{
    if(!impl->sigItemCollapsed){
        stdx::emplace(impl->sigItemCollapsed);
        connect(this, SIGNAL(itemCollapsed(QTreeWidgetItem*)),
                this, SLOT(onItemCollapsed(QTreeWidgetItem*)));
    }
    return *impl->sigItemCollapsed;
}


SignalProxy<void(QTreeWidgetItem* item, int column)> TreeWidget::sigItemDoubleClicked()
{
    if(!impl->sigItemDoubleClicked){
        stdx::emplace(impl->sigItemDoubleClicked);
        connect(this, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)),
                this, SLOT(onItemDoubleClicked(QTreeWidgetItem*, int)));
    }
    return *impl->sigItemDoubleClicked;
}


SignalProxy<void(QTreeWidgetItem* item, int column)> TreeWidget::sigItemEntered()
{
    if(!impl->sigItemEntered){
        stdx::emplace(impl->sigItemEntered);
        connect(this, SIGNAL(itemEntered(QTreeWidgetItem*, int)),
                this, SLOT(onItemEntered(QTreeWidgetItem*, int)));
    }
    return *impl->sigItemEntered;
}


SignalProxy<void(QTreeWidgetItem* item)> TreeWidget::sigItemExpanded()
{
    if(!impl->sigItemExpanded){
        stdx::emplace(impl->sigItemExpanded);
        connect(this, SIGNAL(itemExpanded(QTreeWidgetItem*)),
                this, SLOT(onItemExpanded(QTreeWidgetItem*)));
    }
    return *impl->sigItemExpanded;
}


SignalProxy<void(QTreeWidgetItem* item, int column)> TreeWidget::sigItemPressed()
{
    if(!impl->sigItemPressed){
        stdx::emplace(impl->sigItemPressed);
        connect(this, SIGNAL(itemPressed(QTreeWidgetItem*, int)),
                this, SLOT(onItemPressed(QTreeWidgetItem*, int)));
    }
    return *impl->sigItemPressed;
}


SignalProxy<void()> TreeWidget::sigItemSelectionChanged()
{
    if(!impl->sigItemSelectionChanged){
        stdx::emplace(impl->sigItemSelectionChanged);
        connect(this, SIGNAL(itemSelectionChanged()),
                this, SLOT(onItemSelectionChanged()));
    }
    return *impl->sigItemSelectionChanged;
}


SignalProxy<void(const QModelIndex &parent, int first, int last)> TreeWidget::sigRowsAboutToBeRemoved()
{
    if(!impl->sigRowsAboutToBeRemoved){
        stdx::emplace(impl->sigRowsAboutToBeRemoved);
        QObject::connect(model(), SIGNAL(rowsAboutToBeRemoved(const QModelIndex&, int, int)),
                         this, SLOT(onRowsAboutToBeRemoved(const QModelIndex&, int, int)));
    }
    return *impl->sigRowsAboutToBeRemoved;
}


SignalProxy<void(const QModelIndex &parent, int first, int last)> TreeWidget::sigRowsRemoved()
{
    if(!impl->sigRowsRemoved){
        stdx::emplace(impl->sigRowsRemoved);
        QObject::connect(model(), SIGNAL(rowsRemoved(const QModelIndex&, int, int)),
                         this, SLOT(onRowsRemoved(const QModelIndex&, int, int)));
    }
    return *impl->sigRowsRemoved;
}


SignalProxy<void(const QModelIndex &parent, int first, int last)> TreeWidget::sigRowsInserted()
{
    if(!impl->sigRowsInserted){
        stdx::emplace(impl->sigRowsInserted);
        QObject::connect(model(), SIGNAL(rowsInserted(const QModelIndex&, int, int)),
                         this, SLOT(onRowsInserted(const QModelIndex&, int, int)));
    }
    return *impl->sigRowsInserted;
}


SignalProxy<void(int logicalIndex, int oldSize, int newSize)> TreeWidget::sigSectionResized()
{
    if(!impl->sigSectionResized){
        stdx::emplace(impl->sigSectionResized);
        QObject::connect(header(), SIGNAL(sectionResized(int, int, int)),
                         this, SLOT(onSectionResized(int, int, int)));
    }
    return *impl->sigSectionResized;
}


void TreeWidget::onCurrentItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous)
{
    (*impl->sigCurrentItemChanged)(current, previous);
}


void TreeWidget::onItemActivated(QTreeWidgetItem* item, int column)
{
    (*impl->sigItemActivated)(item, column);
}

void TreeWidget::onItemChanged(QTreeWidgetItem* item, int column)
{
    (*impl->sigItemChanged)(item, column);
}

void TreeWidget::onItemClicked(QTreeWidgetItem* item, int column)
{
    (*impl->sigItemClicked)(item, column);
}


void TreeWidget::onItemCollapsed(QTreeWidgetItem* item)
{
    (*impl->sigItemCollapsed)(item);
}


void TreeWidget::onItemDoubleClicked(QTreeWidgetItem* item, int column)
{
    (*impl->sigItemDoubleClicked)(item, column);
}


void TreeWidget::onItemEntered(QTreeWidgetItem* item, int column)
{
    (*impl->sigItemEntered)(item, column);
}


void TreeWidget::onItemExpanded(QTreeWidgetItem* item)
{
    (*impl->sigItemExpanded)(item);
}


void TreeWidget::onItemPressed(QTreeWidgetItem* item, int column)
{
    (*impl->sigItemPressed)(item, column);
}


void TreeWidget::onItemSelectionChanged(void)
{
    (*impl->sigItemSelectionChanged)();
}


void TreeWidget::onRowsAboutToBeRemoved(const QModelIndex& parent, int first, int last)
{
    (*impl->sigRowsAboutToBeRemoved)(parent, first, last);
}


void TreeWidget::onRowsRemoved(const QModelIndex& parent, int first, int last)
{
    (*impl->sigRowsRemoved)(parent, first, last);
}


void TreeWidget::onRowsInserted(const QModelIndex& parent, int first, int last)
{
    (*impl->sigRowsInserted)(parent, first, last);
}


void TreeWidget::onSectionResized(int logicalIndex, int oldSize, int newSize)
{
    (*impl->sigSectionResized)(logicalIndex, oldSize, newSize);
}
