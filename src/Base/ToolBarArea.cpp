/**
   @author Shin'ichiro Nakaoka
*/

#include "ToolBarArea.h"
#include "Separator.h"
#include "ToolBar.h"
#include "MainWindow.h"
#include "LazyCaller.h"
#include "Menu.h"
#include <cnoid/ValueTree>
#include <QResizeEvent>
#include <QApplication>
#include <deque>
#include <vector>
#include <list>
#include <set>
#include <limits>
#include <algorithm>
#include <iostream>

using namespace std;
using namespace cnoid;
using namespace std::placeholders;

namespace {

const bool DEBUG_MODE = false;

typedef std::vector<ToolBar*> ToolBarArray;


class ToolBarRow
{
public:
    ToolBarRow(ToolBarArea* toolBarArea) :
        toolBarArea(toolBarArea),
        separator(toolBarArea) {
        height = 0;
        separator.hide();
    }

    ToolBarArea* toolBarArea;
    ToolBarArray toolBars;
    HSeparator separator;
    int height;
};
typedef std::shared_ptr<ToolBarRow> ToolBarRowPtr;


// for archiving
struct LayoutState
{
public:
    string name;
    int desiredX;
    int layoutPriority;
};

typedef list<LayoutState> RowLayoutState;
typedef list<RowLayoutState> AreaLayoutState;
}


namespace cnoid {

class ToolBarAreaImpl
{
public:
    ToolBarAreaImpl(ToolBarArea* self);
    ~ToolBarAreaImpl();

    bool addToolBar(ToolBar* toolBar);
    void removeToolBar(ToolBar* toolBar);
    void hideToolBar(ToolBar* toolBar);
    void setVisibilityMenuItems(Menu* menu);
    void onVisiblityCheckToggled(ToolBar* toolBar, bool on);
    void storeLayout(MappingPtr& archive);
    void restoreLayout(MappingPtr& archive);
    void resetLayout(MappingPtr& archive);
    void dragToolBar(ToolBar* toolBar, const QPoint& globalPos);
        
    void setNewToolBars();
    void expandStrechableBars
    (ToolBarRowPtr& row, int portion, int shift, int barIndex, int numStrechables, int lastSpace);

    void useSpace(ToolBarRowPtr& row, int space, int shift, int barIndex, int numStrechables, int allspace );
    void setNewToolBar(ToolBar* toolBar, vector<int>& numStrechablesOfRow);
    void layoutToolBars();
    void layoutToolBarRow(ToolBarRowPtr toolBarRow, int& io_rowTop, bool isNewBottomRow);
    void layoutToolBarRowWithDraggedToolBar(ToolBarArray& toolBars, int rowTop, int rowHeight);
    int normalizeLayoutPriorities(ToolBarArray& toolBars);
    void layoutToolBarRowPart(ToolBarArray& toolBars, int rowTop, int rowHeight, int partLeft, int partRight);

    void resizeEvent(QResizeEvent* event);

    ToolBarArea* self;

    bool isBeforeDoingInitialLayout;
    
    int defaultOrderIndex;
    
    struct DefaultOrderCmp {
        bool operator() (ToolBar* bar1, ToolBar* bar2) {
            return (bar1->defaultOrderIndex < bar2->defaultOrderIndex);
        }
    };
        
    set<ToolBar*> toolBars;
    ToolBarArray newToolBarsToShow;
    deque<ToolBarRowPtr> toolBarRows;
    ToolBarRowPtr spilledToolBarRow;
    MappingPtr initialLayout;

    int areaHeight;
    int prevAreaHeight;

    ToolBar* draggedToolBar;
    bool draggedToolBarHasNotBeenInserted;
    int dragY;

    LazyCaller layoutToolBarsLater;
};

}


ToolBarArea::ToolBarArea(QWidget* parent)
    : QWidget(parent)
{
    impl = new ToolBarAreaImpl(this);
}


ToolBarAreaImpl::ToolBarAreaImpl(ToolBarArea* self)
    : self(self),
      spilledToolBarRow(new ToolBarRow(self)),
      layoutToolBarsLater(std::bind(&ToolBarAreaImpl::layoutToolBars, this))
{
    isBeforeDoingInitialLayout = true;
    defaultOrderIndex = 0;
    
    draggedToolBar = 0;
    draggedToolBarHasNotBeenInserted = false;

    areaHeight = 0;
    prevAreaHeight = 0;
}


ToolBarArea::~ToolBarArea()
{
    delete impl;
}


ToolBarAreaImpl::~ToolBarAreaImpl()
{

}


void ToolBarArea::getAllToolBars(std::vector<ToolBar*>& out_toolBars)
{
    out_toolBars.clear();
    for(set<ToolBar*>::iterator p = impl->toolBars.begin(); p != impl->toolBars.end(); ++p){
        out_toolBars.push_back(*p);
    }
}


void ToolBarArea::getVisibleToolBars(std::vector<ToolBar*>& out_toolBars)
{
    out_toolBars.clear();
    
    for(size_t i=0; i < impl->toolBarRows.size(); ++i){
        ToolBarRowPtr& row = impl->toolBarRows[i];
        vector<ToolBar*>& toolBars = row->toolBars;
        for(size_t j=0; j < toolBars.size(); ++j){
            out_toolBars.push_back(toolBars[j]);
        }
    }
}


bool ToolBarArea::addToolBar(ToolBar* toolBar)
{
    return impl->addToolBar(toolBar);
}


bool ToolBarAreaImpl::addToolBar(ToolBar* toolBar)
{
    if(DEBUG_MODE){
        cout << "ToolBarAreaImpl::addToolBar()" << endl;
    }
        
    if(toolBar){
        set<ToolBar*>::iterator p = toolBars.find(toolBar);
        if(p == toolBars.end()){
            toolBars.insert(toolBar);

            toolBar->toolBarArea_ = self;
            toolBar->defaultOrderIndex = defaultOrderIndex++;
            toolBar->hide();
            toolBar->setParent(self);

            if(toolBar->isVisibleByDefault()){
                newToolBarsToShow.push_back(toolBar);
            }
            if(!isBeforeDoingInitialLayout){
                layoutToolBarsLater();
            }
            return true;
        }
    }
    return false;
}


void ToolBarArea::removeToolBar(ToolBar* toolBar)
{
    if(toolBar){
        impl->removeToolBar(toolBar);
    }
}


void ToolBarAreaImpl::removeToolBar(ToolBar* toolBar)
{
    set<ToolBar*>::iterator p = toolBars.find(toolBar);
    if(p != toolBars.end()){
        hideToolBar(toolBar);
        toolBar->setParent(0);
        toolBar->toolBarArea_ = 0;
        toolBars.erase(p);
    }
}


void ToolBarAreaImpl::hideToolBar(ToolBar* toolBar)
{
    if(toolBar->isVisible()){
        for(size_t i=0; i < toolBarRows.size(); ++i){
            ToolBarRowPtr row = toolBarRows[i];
            vector<ToolBar*>& toolBars = row->toolBars;
            for(size_t j=0; j < toolBars.size(); ++j){
                if(toolBars[j] == toolBar){
                    toolBar->hide();
                    toolBars.erase(toolBars.begin() + j);
                    if(!isBeforeDoingInitialLayout){
                        layoutToolBarsLater();
                    }
                    return;
                }
            }
        }
    }
}


void ToolBarArea::setVisibilityMenuItems(Menu* menu)
{
    impl->setVisibilityMenuItems(menu);
}


void ToolBarAreaImpl::setVisibilityMenuItems(Menu* menu)
{
    menu->clear();

    vector<ToolBar*> sorted(toolBars.size());
    std::copy(toolBars.begin(), toolBars.end(), sorted.begin());
    std::sort(sorted.begin(), sorted.end(), DefaultOrderCmp());
    
    for(size_t i=0; i < sorted.size(); ++i){
        ToolBar* toolBar = sorted[i];
        Action* action = new Action(menu);
        action->setText(toolBar->windowTitle());
        action->setCheckable(true);
        if(toolBar->isVisible()){
            action->setChecked(true);
        }
        action->sigToggled().connect(std::bind(&ToolBarAreaImpl::onVisiblityCheckToggled, this, toolBar, _1));
        menu->addAction(action);
    }
}


void ToolBarAreaImpl::onVisiblityCheckToggled(ToolBar* toolBar, bool on)
{
    if(on){
        newToolBarsToShow.push_back(toolBar);
        layoutToolBarsLater();
    } else {
        hideToolBar(toolBar);
    }
}


void ToolBarArea::setInitialLayout(MappingPtr archive)
{
    if(impl->isBeforeDoingInitialLayout){
        impl->initialLayout = archive;
    }
}


void ToolBarArea::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    impl->resizeEvent(event);
}


void ToolBarAreaImpl::resizeEvent(QResizeEvent* event)
{
    if(DEBUG_MODE){
        cout << "ToolBarAreaImpl::resizeEvent(" <<
            event->size().width() << ", " << event->size().height() << ")";
        cout << ", isVisible =" << self->isVisible();
        cout << ", MainWindows's windowState() = " << MainWindow::instance()->windowState() << endl;
    }

    if(isBeforeDoingInitialLayout){
#ifdef Q_OS_MAC
        bool doLayout = true;
#else
        bool doLayout =
            !(MainWindow::instance()->windowState() & (Qt::WindowMaximized | Qt::WindowFullScreen)) ||
            self->isVisible();
#endif
        if(doLayout){
            isBeforeDoingInitialLayout = false;
            if(initialLayout){
                restoreLayout(initialLayout);
                initialLayout = 0;
            } else {
                layoutToolBars();
            }
        }
    } else {
        if(event->oldSize().width() >= 0 && event->size().width() != event->oldSize().width()){
            layoutToolBars();
        }
    }
}


bool ToolBarArea::event(QEvent* event)
{
    if(false){ // Is this needed?
        if(event->type() == QEvent::LayoutRequest){
            if(DEBUG_MODE){
                cout << "ToolBarArea::event(QEvent::LayoutRequest)" << endl;
            }
            impl->layoutToolBars();
            return true;
        }
    }
    return QWidget::event(event);
}


void ToolBarArea::doInitialLayout()
{
    if(DEBUG_MODE){
        cout << "ToolBarAreaImpl::doInitialLayout()" << endl;
        cout << "width = " << width() << endl;
    }
    
    if(impl->isBeforeDoingInitialLayout){
        impl->isBeforeDoingInitialLayout = false;
        if(impl->initialLayout){
            impl->restoreLayout(impl->initialLayout);
            impl->initialLayout = 0;
        } else {
            impl->layoutToolBars();
        }
    }
}


void ToolBarArea::restoreLayout(MappingPtr archive)
{
    impl->restoreLayout(archive);
}


void ToolBarAreaImpl::restoreLayout(MappingPtr& archive)
{
    if(DEBUG_MODE){
        cout << "ToolBarAreaImpl::restoreLayout()" << endl;
    }

    if(isBeforeDoingInitialLayout){
        initialLayout = archive;
        return;
    }

    const MappingPtr layoutOfToolBars = archive->findMapping("layoutOfToolBars");
    if(!layoutOfToolBars->isValid()){
        layoutToolBars();
        return;
    }

    // make the map from name to toolBar
    typedef map<QString, ToolBar*> NameToToolBarMap;
    NameToToolBarMap nameToToolBarMap;
    for(set<ToolBar*>::iterator p = toolBars.begin(); p != toolBars.end(); ++p){
        ToolBar* toolBar = *p;
        toolBar->hide();
        nameToToolBarMap[toolBar->objectName()] = toolBar;
    }

    newToolBarsToShow.clear();
    
    toolBarRows.clear();

    const Listing* rows = layoutOfToolBars->get("rows").toListing();

    for(int i=0; i < rows->size(); ++i){
        ToolBarRowPtr row(new ToolBarRow(self));
        toolBarRows.push_back(row);
        const Listing* bars = rows->at(i)->toListing();
        for(int j=0; j < bars->size(); ++j){
            const Mapping* state = bars->at(j)->toMapping();
            NameToToolBarMap::iterator it = nameToToolBarMap.find(QString(state->get("name").toString().c_str()));
            if(it != nameToToolBarMap.end()){
                ToolBar* toolBar = it->second;
                row->toolBars.push_back(toolBar);
                toolBar->desiredX = state->get("x").toInt();
                toolBar->layoutPriority = state->get("priority").toInt();
                nameToToolBarMap.erase(it);
            }
        }
        if(row->toolBars.empty()){
            toolBarRows.pop_back();
        }
    }

    layoutToolBars();
}


void ToolBarArea::resetLayout(MappingPtr archive)
{
    impl->resetLayout(archive);
}


void ToolBarAreaImpl::resetLayout(MappingPtr& archive)
{
    if(DEBUG_MODE){
        cout << "ToolBarAreaImpl::resetLayout()" << endl;
    }
    newToolBarsToShow.clear();
    for(set<ToolBar*>::iterator p = toolBars.begin(); p != toolBars.end(); ++p){
        ToolBar* toolBar = *p;
        toolBar->hide();
        if(toolBar->isVisibleByDefault()){
            newToolBarsToShow.push_back(toolBar);
        }
    }
    std::sort(newToolBarsToShow.begin(), newToolBarsToShow.end(), DefaultOrderCmp());
    
    toolBarRows.clear();
    layoutToolBars();
    
    self->removeLayout(archive);
}


void ToolBarArea::removeLayout(MappingPtr archive)
{
    archive->remove("layoutOfToolBars");
}


void ToolBarArea::storeLayout(MappingPtr archive)
{
    impl->storeLayout(archive);
}


void ToolBarAreaImpl::storeLayout(MappingPtr& archive)
{
    Mapping* layoutOfToolBars = archive->createMapping("layoutOfToolBars");
    Listing* rows = layoutOfToolBars->createListing("rows");
    
    for(size_t i=0; i < toolBarRows.size(); ++i){
        ToolBarArray& toolBars = toolBarRows[i]->toolBars;
        if(!toolBars.empty()){
            Listing* bars = new Listing();
            for(ToolBarArray::iterator p = toolBars.begin(); p != toolBars.end(); ++p){
                ToolBar* toolBar = *p;
                Mapping* state = new Mapping();
                state->setFlowStyle(true);
                state->write("name", toolBar->objectName().toStdString(), DOUBLE_QUOTED);
                state->write("x", toolBar->desiredX);
                state->write("priority", toolBar->layoutPriority);
                bars->append(state);
            }
            rows->append(bars);
        }
    }
}


void ToolBarArea::dragToolBar(ToolBar* toolBar, const QPoint& globalPos)
{
    impl->dragToolBar(toolBar, globalPos);
}


void ToolBarAreaImpl::dragToolBar(ToolBar* toolBar, const QPoint& globalPos)
{
    QPoint p = self->mapFromGlobal(globalPos);
        
    draggedToolBar = toolBar;
    draggedToolBarHasNotBeenInserted = true;

    draggedToolBar->desiredX = p.x();
    dragY = p.y();

    if(p.y() < 0){
        toolBarRows.push_front(ToolBarRowPtr(new ToolBarRow(self)));
    }
    
    layoutToolBars();
    draggedToolBar = 0;
}


void ToolBarAreaImpl::setNewToolBars()
{
    vector<int> numStrechablesOfRow(toolBarRows.size(), 0);

    for(size_t i=0; i < newToolBarsToShow.size(); ++i){
        setNewToolBar(newToolBarsToShow[i], numStrechablesOfRow);
    }
    newToolBarsToShow.clear();

    for(size_t i=0; i < numStrechablesOfRow.size(); ++i){
        int numStrechables = numStrechablesOfRow[i];
        if(numStrechables > 0){
            ToolBarRowPtr& row = toolBarRows[i];
            ToolBar* lastToolBar = row->toolBars.back();
            QRect r = lastToolBar->geometry();
            int space = self->width() - (r.x() + r.width());
            if(space > 0){
                expandStrechableBars(row, (space / numStrechables), 0, 0, numStrechables, space);
            }
        }
    }
}


void ToolBarAreaImpl::expandStrechableBars
(ToolBarRowPtr& row, int portion, int shift, int barIndex, int numStrechables, int lastSpace)
{
    ToolBar* bar = row->toolBars[barIndex];
    bar->desiredX += shift;
    if(bar->isStretchable()){
        int addition = (numStrechables > 1) ? portion : lastSpace;
        shift += addition;
        lastSpace -= addition;
        --numStrechables;
    }
    ++barIndex;
    if(barIndex < (int)row->toolBars.size()){
        expandStrechableBars(row, portion, shift, barIndex,  numStrechables, lastSpace);
    }
}


void ToolBarAreaImpl::setNewToolBar(ToolBar* toolBar, vector<int>& numStrechablesOfRow)
{
    if(DEBUG_MODE){
        cout << "ToolBarAreaImpl::setNewToolBar()" << endl;
    }
    
    if(toolBar){

        ToolBarRowPtr toolBarRow;
        int rowIndex = -1;
        int width = toolBar->minimumSizeHint().width();

        for(size_t i=0; i < toolBarRows.size(); ++i){
            ToolBarRowPtr existingRow = toolBarRows[i];
            ToolBar* lastToolBar = existingRow->toolBars.back();
            if(lastToolBar){
                QRect r = lastToolBar->geometry();
                int lastX = r.x() + r.width();
                int lastSpace = self->width() - lastX;
                if(width <= lastSpace){
                    toolBar->desiredX = lastX + 1;
                    if(toolBar->isStretchable()){
                        width = std::min(toolBar->stretchableDefaultWidth(), lastSpace);
                    }
                    toolBarRow = existingRow;
                    rowIndex = i;
                    break;
                }
            }
        }
        if(!toolBarRow){
            toolBar->desiredX = 0;
            rowIndex = toolBarRows.size();
            toolBarRow.reset(new ToolBarRow(self));
            toolBarRows.push_back(toolBarRow);
            numStrechablesOfRow.push_back(0);
        }
        
        toolBarRow->toolBars.push_back(toolBar);
        if(toolBar->isStretchable()){
            numStrechablesOfRow[rowIndex]++;
        }
        toolBar->setGeometry(toolBar->desiredX, 0, width, toolBar->minimumSizeHint().height());
        toolBar->show();
    }
}


void ToolBarArea::layoutToolBars()
{
    impl->layoutToolBars();
}


void ToolBarAreaImpl::layoutToolBars()
{
    if(DEBUG_MODE){
        cout << "ToolBarAreaImpl::layoutToolBars()" << endl;
    }

    if(!newToolBarsToShow.empty()){
        setNewToolBars();
    }

    if(DEBUG_MODE){
        cout << "actually do ToolBarAreaImpl::layoutToolBars()" << endl;
    }
    areaHeight = 0;
    int rowTop = 0;
    spilledToolBarRow->toolBars.clear();
    
    size_t i = 0;
    while(i < toolBarRows.size()){
        layoutToolBarRow(toolBarRows[i], rowTop, false);
        if(!spilledToolBarRow->toolBars.empty()){
            layoutToolBarRow(spilledToolBarRow, rowTop, false);
        }
        if(toolBarRows[i]->toolBars.empty()){
            toolBarRows.erase(toolBarRows.begin() + i);
        } else {
            ++i;
        }
    }
    if(draggedToolBarHasNotBeenInserted){
        ToolBarRowPtr row(new ToolBarRow(self));
        toolBarRows.push_back(row);
        layoutToolBarRow(row, rowTop, true);
    }
    
    areaHeight = rowTop;

    if(areaHeight != prevAreaHeight){
        self->setMinimumHeight(areaHeight);
        prevAreaHeight = areaHeight;
    }

    isBeforeDoingInitialLayout = false;
}


void ToolBarAreaImpl::layoutToolBarRow(ToolBarRowPtr toolBarRow, int& io_rowTop, bool isNewBottomRow)
{
    int rowHeight = 0;

    bool draggedToolBarExists = false;
    ToolBarArray& toolBars = toolBarRow->toolBars;

    // calculate the row height and remove the dragged tool bar if it is originally in this row
    ToolBarArray::iterator p = toolBars.begin();
    while(p != toolBars.end()){
        ToolBar* toolBar = *p;
        if(toolBar == draggedToolBar){
            draggedToolBarExists = true;
            p = toolBars.erase(p);
        } else {
            rowHeight = std::max(rowHeight, toolBar->minimumSizeHint().height());
            ++p;
        }
    }

    // Is the dragged tool bar moving to this row ?
    bool isDraggedToolBarMovingToThisRow = false;
    
    if(draggedToolBarHasNotBeenInserted){
        int bottomBorder = 0;
        if(rowHeight == 0 && draggedToolBarExists){
            int h = draggedToolBar->minimumSizeHint().height();
            bottomBorder = io_rowTop + h * 4 / 5;
        } else {
            bottomBorder = io_rowTop + rowHeight;
        }
        if(dragY < bottomBorder || isNewBottomRow){
            isDraggedToolBarMovingToThisRow = true;
            rowHeight = std::max(rowHeight, draggedToolBar->minimumSizeHint().height());
            layoutToolBarRowWithDraggedToolBar(toolBars, io_rowTop, rowHeight);
        }
    }

    if(toolBars.empty()){
        toolBarRow->separator.hide();
    } else {
        if(!isDraggedToolBarMovingToThisRow){
            layoutToolBarRowPart(toolBars, io_rowTop, rowHeight, 0, self->width());
        }

        io_rowTop += rowHeight;

        HSeparator& sep = toolBarRow->separator;
        int h = sep.sizeHint().height();
        sep.setGeometry(0, io_rowTop, self->width(), h);
        sep.show();
        io_rowTop += sep.sizeHint().height();
    }
}


void ToolBarAreaImpl::layoutToolBarRowWithDraggedToolBar(ToolBarArray& toolBars, int rowTop, int rowHeight)
{
    int maxPriorty = normalizeLayoutPriorities(toolBars);
    draggedToolBar->layoutPriority = maxPriorty + 1;

    for(size_t i=0; i < toolBars.size(); ++i){
        if(draggedToolBar->desiredX <= toolBars[i]->desiredX){
            toolBars.insert(toolBars.begin() + i, draggedToolBar);
            draggedToolBarHasNotBeenInserted = false;
            break;
        }
    }
    if(draggedToolBarHasNotBeenInserted){
        toolBars.push_back(draggedToolBar);
        draggedToolBarHasNotBeenInserted = false;
    }

    layoutToolBarRowPart(toolBars, rowTop, rowHeight, 0, self->width());

    draggedToolBar->desiredX = draggedToolBar->geometry().x();
}


/**
   @return The maximum priority in the normalized priorities
*/
int ToolBarAreaImpl::normalizeLayoutPriorities(ToolBarArray& toolBars)
{
    int maxPriority = 0;

    if(!toolBars.empty()){

        ToolBarArray sortedToolBars(toolBars);
        std::sort(sortedToolBars.begin(), sortedToolBars.end(), ToolBar::LayoutPriorityCmp());

        int prevOldPriority = -1;
        int priority = -1;
        for(size_t i=0; i < sortedToolBars.size(); ++i){
            ToolBar* bar = sortedToolBars[i];
            if(bar->layoutPriority > prevOldPriority){
                prevOldPriority = bar->layoutPriority;
                priority += 1;
            }
            bar->layoutPriority = priority;
            if(priority > maxPriority){
                maxPriority = priority;
            }
        }
    }

    return maxPriority;
}


void ToolBarAreaImpl::layoutToolBarRowPart
(ToolBarArray& toolBars, int rowTop, int rowHeight, int partLeft, int partRight)
{
    // find the index of the tool bar with the maximum priority
    int pivotIndex = 0;
    int maxPriority = -1;
    for(size_t i=0; i < toolBars.size(); ++i){
        if(toolBars[i]->layoutPriority > maxPriority){
            maxPriority = toolBars[i]->layoutPriority;
            pivotIndex = i;
        }
    }

    ToolBarArray leftPartToolBars;
    int pivotAreaLeft = partLeft;
    for(int i=0; i < pivotIndex; ++i){
        leftPartToolBars.push_back(toolBars[i]);
        pivotAreaLeft += toolBars[i]->minimumSizeHint().width();
    }

    ToolBarArray rightPartToolBars;
    int pivotAreaRight = partRight;
    for(size_t i = pivotIndex + 1; i < toolBars.size(); ++i){
        rightPartToolBars.push_back(toolBars[i]);
        pivotAreaRight -= toolBars[i]->minimumSizeHint().width();
    }

    ToolBar* pivotToolBar = toolBars[pivotIndex];
    int x = pivotToolBar->desiredX;
    const QSize size = pivotToolBar->minimumSizeHint();
    int width = size.width();
    if(width < 0){
        width = 0;
    }
    if(x + width > pivotAreaRight){
        x = pivotAreaRight - width;
    }
    if(x < pivotAreaLeft){
        x = pivotAreaLeft;
    }

    if(!leftPartToolBars.empty()){
        layoutToolBarRowPart(leftPartToolBars, rowTop, rowHeight, partLeft, x);
        QRect r = leftPartToolBars.back()->geometry();
        int leftPartRight = r.x() + r.width();
        if(x <= leftPartRight){
            x = leftPartRight + 1;
        }
    }

    int rightPartLeft = partRight;
    if(!rightPartToolBars.empty()){
        layoutToolBarRowPart(rightPartToolBars, rowTop, rowHeight, x + width, partRight);
        rightPartLeft = rightPartToolBars.front()->geometry().x();
    }

    if(pivotToolBar->isStretchable()){
        width = pivotToolBar->maximumWidth();
        int possibleWidth = rightPartLeft - x;
        if(width > possibleWidth){
            width = possibleWidth;
        }
    }

    int height = (size.height() >= 0) ? size.height() : rowHeight;
    int y = rowTop + (rowHeight - height) / 2;

    pivotToolBar->setGeometry(x, y, width, height);
    pivotToolBar->show();
}
