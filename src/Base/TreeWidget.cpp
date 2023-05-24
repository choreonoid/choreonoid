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
    
    stdx::optional<Signal<void(const QPoint& pos)>> sigCustomContextMenuRequested;

    Impl();
};

}


TreeWidget::TreeWidget(QWidget* parent)
    : QTreeWidget(parent)
{
    impl = new Impl;
    isUserInputEnabled_ = true;
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
        connect(this, (void(QTreeWidget::*)(QTreeWidgetItem*, QTreeWidgetItem*)) &QTreeWidget::currentItemChanged,
                [this](QTreeWidgetItem* current, QTreeWidgetItem* previous){
                    (*impl->sigCurrentItemChanged)(current, previous);
                });
    }
    return *impl->sigCurrentItemChanged;
}


SignalProxy<void(QTreeWidgetItem* item, int column)> TreeWidget::sigItemActivated()
{
    if(!impl->sigItemActivated){
        stdx::emplace(impl->sigItemActivated);
        connect(this, (void(QTreeWidget::*)(QTreeWidgetItem*, int)) &QTreeWidget::itemActivated,
                [this](QTreeWidgetItem* item, int column){
                    (*impl->sigItemActivated)(item, column);
                });
    }
    return *impl->sigItemActivated;
}


SignalProxy<void(QTreeWidgetItem* item, int column)> TreeWidget::sigItemChanged()
{
    if(!impl->sigItemChanged){
        stdx::emplace(impl->sigItemChanged);
        connect(this, (void(QTreeWidget::*)(QTreeWidgetItem*, int)) &QTreeWidget::itemChanged,
                [this](QTreeWidgetItem* item, int column){
                    (*impl->sigItemChanged)(item, column);
                });
    }
    return *impl->sigItemChanged;
}


SignalProxy<void(QTreeWidgetItem* item, int column)> TreeWidget::sigItemClicked()
{
    if(!impl->sigItemClicked){
        stdx::emplace(impl->sigItemClicked);
        connect(this, (void(QTreeWidget::*)(QTreeWidgetItem*, int)) &QTreeWidget::itemClicked,
                [this](QTreeWidgetItem* item, int column){
                    (*impl->sigItemClicked)(item, column);
                });
    }
    return *impl->sigItemClicked;
}


SignalProxy<void(QTreeWidgetItem* item)> TreeWidget::sigItemCollapsed()
{
    if(!impl->sigItemCollapsed){
        stdx::emplace(impl->sigItemCollapsed);
        connect(this, (void(QTreeWidget::*)(QTreeWidgetItem*)) &QTreeWidget::itemCollapsed,
                [this](QTreeWidgetItem* item){
                    (*impl->sigItemCollapsed)(item);
                });
    }
    return *impl->sigItemCollapsed;
}


SignalProxy<void(QTreeWidgetItem* item, int column)> TreeWidget::sigItemDoubleClicked()
{
    if(!impl->sigItemDoubleClicked){
        stdx::emplace(impl->sigItemDoubleClicked);
        connect(this, (void(QTreeWidget::*)(QTreeWidgetItem*, int)) &QTreeWidget::itemDoubleClicked,
                [this](QTreeWidgetItem* item, int column){
                    (*impl->sigItemDoubleClicked)(item, column);
                });
    }
    return *impl->sigItemDoubleClicked;
}


SignalProxy<void(QTreeWidgetItem* item, int column)> TreeWidget::sigItemEntered()
{
    if(!impl->sigItemEntered){
        stdx::emplace(impl->sigItemEntered);
        connect(this, (void(QTreeWidget::*)(QTreeWidgetItem*, int)) &QTreeWidget::itemEntered,
                [this](QTreeWidgetItem* item, int column){
                    (*impl->sigItemEntered)(item, column);
                });
    }
    return *impl->sigItemEntered;
}


SignalProxy<void(QTreeWidgetItem* item)> TreeWidget::sigItemExpanded()
{
    if(!impl->sigItemExpanded){
        stdx::emplace(impl->sigItemExpanded);
        connect(this, (void(QTreeWidget::*)(QTreeWidgetItem*)) &QTreeWidget::itemExpanded,
                [this](QTreeWidgetItem* item){
                    (*impl->sigItemExpanded)(item);
                });
    }
    return *impl->sigItemExpanded;
}


SignalProxy<void(QTreeWidgetItem* item, int column)> TreeWidget::sigItemPressed()
{
    if(!impl->sigItemPressed){
        stdx::emplace(impl->sigItemPressed);
        connect(this, (void(QTreeWidget::*)(QTreeWidgetItem*, int)) &QTreeWidget::itemPressed,
                [this](QTreeWidgetItem* item, int column){
                    (*impl->sigItemPressed)(item, column);
                });
    }
    return *impl->sigItemPressed;
}


SignalProxy<void()> TreeWidget::sigItemSelectionChanged()
{
    if(!impl->sigItemSelectionChanged){
        stdx::emplace(impl->sigItemSelectionChanged);
        connect(this, (void(QTreeWidget::*)()) &QTreeWidget::itemSelectionChanged,
                [this](){ (*impl->sigItemSelectionChanged)(); });
    }
    return *impl->sigItemSelectionChanged;
}


SignalProxy<void(const QModelIndex& parent, int first, int last)> TreeWidget::sigRowsAboutToBeRemoved()
{
    if(!impl->sigRowsAboutToBeRemoved){
        stdx::emplace(impl->sigRowsAboutToBeRemoved);
        connect(model(), (void(QAbstractItemModel::*)(const QModelIndex&, int, int)) &QAbstractItemModel::rowsAboutToBeRemoved,
                [this](const QModelIndex& parent, int first, int last){
                    (*impl->sigRowsAboutToBeRemoved)(parent, first, last);
                });
    }
    return *impl->sigRowsAboutToBeRemoved;
}


SignalProxy<void(const QModelIndex& parent, int first, int last)> TreeWidget::sigRowsRemoved()
{
    if(!impl->sigRowsRemoved){
        stdx::emplace(impl->sigRowsRemoved);
        connect(model(), (void(QAbstractItemModel::*)(const QModelIndex&, int, int)) &QAbstractItemModel::rowsRemoved,
                [this](const QModelIndex& parent, int first, int last){
                    (*impl->sigRowsRemoved)(parent, first, last);
                });
    }
    return *impl->sigRowsRemoved;
}


SignalProxy<void(const QModelIndex &parent, int first, int last)> TreeWidget::sigRowsInserted()
{
    if(!impl->sigRowsInserted){
        stdx::emplace(impl->sigRowsInserted);
        connect(model(), (void(QAbstractItemModel::*)(const QModelIndex&, int, int)) &QAbstractItemModel::rowsInserted,
                [this](const QModelIndex& parent, int first, int last){
                    (*impl->sigRowsInserted)(parent, first, last);
                });
    }
    return *impl->sigRowsInserted;
}


SignalProxy<void(int logicalIndex, int oldSize, int newSize)> TreeWidget::sigSectionResized()
{
    if(!impl->sigSectionResized){
        stdx::emplace(impl->sigSectionResized);
        connect(header(), (void(QHeaderView::*)(int, int, int)) &QHeaderView::sectionResized,
                [this](int logicalIndex, int oldSize, int newSize){
                    (*impl->sigSectionResized)(logicalIndex, oldSize, newSize);
                });
    }
    return *impl->sigSectionResized;
}


SignalProxy<void(const QPoint& pos)> TreeWidget::sigCustomContextMenuRequested()
{
    if(!impl->sigCustomContextMenuRequested){
        stdx::emplace(impl->sigCustomContextMenuRequested);
        connect(this, (void(QWidget::*)(const QPoint& pos)) &QWidget::customContextMenuRequested,
                [this](const QPoint& pos){
                    (*impl->sigCustomContextMenuRequested)(pos);
                });
    }
    return *impl->sigCustomContextMenuRequested;
}


void TreeWidget::keyPressEvent(QKeyEvent* event)
{
    if(isUserInputEnabled_){
        QTreeWidget::keyPressEvent(event);
    }
}


void TreeWidget::mousePressEvent(QMouseEvent* event)
{
    if(isUserInputEnabled_){
        QTreeWidget::mousePressEvent(event);
    }
}


void TreeWidget::wheelEvent(QWheelEvent* event)
{
    if(isUserInputEnabled_){
        QTreeWidget::wheelEvent(event);
    }
}
