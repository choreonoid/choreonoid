/**
   @author Shin'ichiro Nakaoka
*/

#include "TreeWidget.h"
#include <QHeaderView>
#include <QPainter>
#include <QPen>

using namespace cnoid;

TreeWidget::TreeWidget(QWidget* parent)
    : QTreeWidget(parent)
{
    connect(this, SIGNAL(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)),
            this, SLOT(onCurrentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)));
    
    connect(this, SIGNAL(itemActivated(QTreeWidgetItem*, int)),
            this, SLOT(onItemActivated(QTreeWidgetItem*, int)));

    connect(this, SIGNAL(itemChanged(QTreeWidgetItem*, int)),
            this, SLOT(onItemChanged(QTreeWidgetItem*, int)));

    connect(this, SIGNAL(itemClicked(QTreeWidgetItem*, int)),
            this, SLOT(onItemClicked(QTreeWidgetItem*, int)));

    connect(this, SIGNAL(itemCollapsed(QTreeWidgetItem*)),
            this, SLOT(onItemCollapsed(QTreeWidgetItem*)));
    
    connect(this, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)),
            this, SLOT(onItemDoubleClicked(QTreeWidgetItem*, int)));

    connect(this, SIGNAL(itemEntered(QTreeWidgetItem*, int)),
            this, SLOT(onItemEntered(QTreeWidgetItem*, int)));

    connect(this, SIGNAL(itemExpanded(QTreeWidgetItem*)),
            this, SLOT(onItemExpanded(QTreeWidgetItem*)));

    connect(this, SIGNAL(itemPressed(QTreeWidgetItem*, int)),
            this, SLOT(onItemPressed(QTreeWidgetItem*, int)));
    
    connect(this, SIGNAL(itemSelectionChanged()),
            this, SLOT(onItemSelectionChanged()));

    isVerticalGridLineShown = false;
}


void TreeWidget::setHeaderSectionResizeMode(int column, QHeaderView::ResizeMode mode)
{
    header()->setSectionResizeMode(column, mode);
}
    

void TreeWidget::setVerticalGridLineShown(bool on)
{
    if(on){
        QStyleOptionViewItem option;
        gridColorRGB = style()->styleHint(QStyle::SH_Table_GridLineColor, &option);
    }
    isVerticalGridLineShown = on;
}
        

TreeWidget::~TreeWidget()
{

}


void TreeWidget::paintEvent(QPaintEvent* event)
{
    QTreeWidget::paintEvent(event);

    if(isVerticalGridLineShown && topLevelItemCount()){
        QHeaderView* hv = header();
        QPainter painter(viewport());
        QPen oldPen = painter.pen();
        painter.setPen(QPen(QColor::fromRgb(gridColorRGB)));
    
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


void TreeWidget::onCurrentItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous)
{
    sigCurrentItemChanged_(current, previous);
}


void TreeWidget::onItemActivated(QTreeWidgetItem* item, int column)
{
    sigItemActivated_(item, column);
}

void TreeWidget::onItemChanged(QTreeWidgetItem* item, int column)
{
    sigItemChanged_(item, column);
}

void TreeWidget::onItemClicked(QTreeWidgetItem* item, int column)
{
    sigItemClicked_(item, column);
}


void TreeWidget::onItemCollapsed(QTreeWidgetItem* item)
{
    sigItemCollapsed_(item);
}


void TreeWidget::onItemDoubleClicked(QTreeWidgetItem* item, int column)
{
    sigItemDoubleClicked_(item, column);
}


void TreeWidget::onItemEntered(QTreeWidgetItem* item, int column)
{
    sigItemEntered_(item, column);
}


void TreeWidget::onItemExpanded(QTreeWidgetItem* item)
{
    sigItemExpanded_(item);
}


void TreeWidget::onItemPressed(QTreeWidgetItem* item, int column)
{
    sigItemPressed_(item, column);
}


void TreeWidget::onItemSelectionChanged(void)
{
    sigItemSelectionChanged_();
}
