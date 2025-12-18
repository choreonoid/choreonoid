#include "TreeWidget.h"
#include <QHeaderView>
#include <QPainter>
#include <QPen>
#include <optional>

using namespace cnoid;

namespace cnoid {

class TreeWidget::Impl
{
public:
    int gridColorRGB;
    bool isVerticalGridLineShown;
    
    std::optional<Signal<void(QTreeWidgetItem* current, QTreeWidgetItem* previous)>> sigCurrentItemChanged;
    std::optional<Signal<void(QTreeWidgetItem* item, int column)>> sigItemActivated;
    std::optional<Signal<void(QTreeWidgetItem* item, int column)>> sigItemChanged;
    std::optional<Signal<void(QTreeWidgetItem* item, int column)>> sigItemClicked;
    std::optional<Signal<void(QTreeWidgetItem* item)>> sigItemCollapsed;
    std::optional<Signal<void(QTreeWidgetItem* item, int column)>> sigItemDoubleClicked;
    std::optional<Signal<void(QTreeWidgetItem* item, int column)>> sigItemEntered;
    std::optional<Signal<void(QTreeWidgetItem* item)>> sigItemExpanded;
    std::optional<Signal<void(QTreeWidgetItem* item, int column)>> sigItemPressed;
    std::optional<Signal<void()>> sigItemSelectionChanged;

    std::optional<Signal<void(const QModelIndex& parent, int first, int last)>> sigRowsAboutToBeRemoved;
    std::optional<Signal<void(const QModelIndex& parent, int first, int last)>> sigRowsRemoved;
    std::optional<Signal<void(const QModelIndex& parent, int first, int last)>> sigRowsInserted;

    std::optional<Signal<void(int logicalIndex, int oldSize, int newSize)>> sigSectionResized;
    
    std::optional<Signal<void(const QPoint& pos)>> sigCustomContextMenuRequested;

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
        impl->sigCurrentItemChanged.emplace();
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
        impl->sigItemActivated.emplace();
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
        impl->sigItemChanged.emplace();
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
        impl->sigItemClicked.emplace();
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
        impl->sigItemCollapsed.emplace();
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
        impl->sigItemDoubleClicked.emplace();
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
        impl->sigItemEntered.emplace();
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
        impl->sigItemExpanded.emplace();
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
        impl->sigItemPressed.emplace();
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
        impl->sigItemSelectionChanged.emplace();
        connect(this, (void(QTreeWidget::*)()) &QTreeWidget::itemSelectionChanged,
                [this](){ (*impl->sigItemSelectionChanged)(); });
    }
    return *impl->sigItemSelectionChanged;
}


SignalProxy<void(const QModelIndex& parent, int first, int last)> TreeWidget::sigRowsAboutToBeRemoved()
{
    if(!impl->sigRowsAboutToBeRemoved){
        impl->sigRowsAboutToBeRemoved.emplace();
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
        impl->sigRowsRemoved.emplace();
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
        impl->sigRowsInserted.emplace();
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
        impl->sigSectionResized.emplace();
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
        impl->sigCustomContextMenuRequested.emplace();
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
