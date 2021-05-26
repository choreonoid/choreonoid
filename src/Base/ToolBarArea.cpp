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
#include <deque>
#include <vector>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

const bool DEBUG_MODE = false;

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
    vector<ToolBar*> toolBars;
    HSeparator separator;
    int height;
};
typedef std::shared_ptr<ToolBarRow> ToolBarRowPtr;

}

namespace cnoid {

class ToolBarArea::Impl
{
public:
    Impl(ToolBarArea* self);
    ~Impl();

    void addToolBar(ToolBar* toolBar);
    void removeToolBar(ToolBar* toolBar);
    void hideToolBar(ToolBar* toolBar);
    void setVisibilityMenuItems(Menu* menu);
    void onVisiblityCheckToggled(ToolBar* toolBar, bool on);
    void storeLayout(Mapping* archive);
    void restoreLayout(Mapping* archive);
    void resetLayout(Mapping* archive);
    void dragToolBar(ToolBar* toolBar, const QPoint& globalPos);
        
    void setNewToolBars();
    void expandStrechableBars(
        ToolBarRowPtr& row, int portion, int shift, int barIndex, int numStrechables, int lastSpace);

    void setNewToolBar(ToolBar* toolBar, vector<int>& numStrechablesOfRow);
    void layoutToolBars();
    void layoutToolBarRow(ToolBarRowPtr toolBarRow, int& io_rowTop, bool isNewBottomRow);
    void layoutToolBarRowWithDraggedToolBar(vector<ToolBar*>& toolBars, int rowTop, int rowHeight);
    int normalizeLayoutPriorities(vector<ToolBar*>& toolBars);
    void layoutToolBarRowPart(vector<ToolBar*>& toolBars, int rowTop, int rowHeight, int partLeft, int partRight);

    void resizeEvent(QResizeEvent* event);

    ToolBarArea* self;

    bool isBeforeDoingInitialLayout;
    
    vector<ToolBar*> registeredToolBars;
    vector<ToolBar*> newToolBarsToShow;
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
    impl = new Impl(this);
}


ToolBarArea::Impl::Impl(ToolBarArea* self)
    : self(self),
      spilledToolBarRow(new ToolBarRow(self)),
      layoutToolBarsLater([this](){ layoutToolBars(); })
{
    isBeforeDoingInitialLayout = true;
    draggedToolBar = nullptr;
    draggedToolBarHasNotBeenInserted = false;
    areaHeight = 0;
    prevAreaHeight = 0;
}


ToolBarArea::~ToolBarArea()
{
    delete impl;
}


ToolBarArea::Impl::~Impl()
{

}


std::vector<ToolBar*> ToolBarArea::toolBars() const
{
    return impl->registeredToolBars;
}

    
std::vector<ToolBar*> ToolBarArea::visibleToolBars() const
{
    vector<ToolBar*> bars;
    bars.reserve(impl->registeredToolBars.size());

    for(size_t i=0; i < impl->toolBarRows.size(); ++i){
        ToolBarRowPtr& row = impl->toolBarRows[i];
        for(auto& bar : row->toolBars){
            bars.push_back(bar);
        }
    }

    return bars;
}


void ToolBarArea::addToolBar(ToolBar* toolBar)
{
    impl->addToolBar(toolBar);
}


void ToolBarArea::Impl::addToolBar(ToolBar* toolBar)
{
    if(DEBUG_MODE){
        cout << "ToolBarArea::Impl::addToolBar()" << endl;
    }
        
    if(toolBar){
        registeredToolBars.push_back(toolBar);
        
        toolBar->toolBarArea_ = self;
        toolBar->hide();
        toolBar->setParent(self);

        if(toolBar->isVisibleByDefault()){
            newToolBarsToShow.push_back(toolBar);
        }
        if(!isBeforeDoingInitialLayout){
            layoutToolBarsLater();
        }
    }
}


void ToolBarArea::removeToolBar(ToolBar* toolBar)
{
    if(toolBar){
        impl->removeToolBar(toolBar);
    }
}


void ToolBarArea::Impl::removeToolBar(ToolBar* toolBar)
{
    auto p = std::find(registeredToolBars.begin(), registeredToolBars.end(), toolBar);
    if(p != registeredToolBars.end()){
        hideToolBar(toolBar);
        toolBar->setParent(nullptr);
        toolBar->toolBarArea_ = nullptr;
        registeredToolBars.erase(p);
    }
}


void ToolBarArea::Impl::hideToolBar(ToolBar* toolBar)
{
    if(toolBar->isVisible()){
        for(size_t i=0; i < toolBarRows.size(); ++i){
            auto row = toolBarRows[i];
            auto& toolBars = row->toolBars;
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


void ToolBarArea::Impl::setVisibilityMenuItems(Menu* menu)
{
    menu->clear();

    for(auto& toolBar : registeredToolBars){
        Action* action = new Action(menu);
        action->setText(toolBar->windowTitle());
        action->setCheckable(true);
        if(toolBar->isVisible()){
            action->setChecked(true);
        }
        action->sigToggled().connect([=](bool on){ onVisiblityCheckToggled(toolBar, on); });
        menu->addAction(action);
    }
}


void ToolBarArea::Impl::onVisiblityCheckToggled(ToolBar* toolBar, bool on)
{
    if(on){
        newToolBarsToShow.push_back(toolBar);
        layoutToolBarsLater();
    } else {
        hideToolBar(toolBar);
    }
}


void ToolBarArea::setInitialLayout(Mapping* archive)
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


void ToolBarArea::Impl::resizeEvent(QResizeEvent* event)
{
    if(DEBUG_MODE){
        cout << "ToolBarArea::Impl::resizeEvent(" <<
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
                initialLayout.reset();
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
        cout << "ToolBarArea::Impl::doInitialLayout()" << endl;
        cout << "width = " << width() << endl;
    }
    
    if(impl->isBeforeDoingInitialLayout){
        impl->isBeforeDoingInitialLayout = false;
        if(impl->initialLayout){
            impl->restoreLayout(impl->initialLayout);
            impl->initialLayout.reset();
        } else {
            impl->layoutToolBars();
        }
    }
}


void ToolBarArea::restoreLayout(Mapping* archive)
{
    impl->restoreLayout(archive);
}


void ToolBarArea::Impl::restoreLayout(Mapping* archive)
{
    if(DEBUG_MODE){
        cout << "ToolBarArea::Impl::restoreLayout()" << endl;
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
    map<string, ToolBar*> nameToToolBarMap;
    for(auto& toolBar : registeredToolBars){
        toolBar->hide();
        nameToToolBarMap[toolBar->objectName().toStdString()] = toolBar;
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
            auto it = nameToToolBarMap.find(state->get("name").toString());
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

    if(!nameToToolBarMap.empty()){
        auto& hidden = *layoutOfToolBars->findListing("hidden");
        if(hidden.isValid()){
            for(auto& node : hidden){
                nameToToolBarMap.erase(node->toString());
            }
        }
        if(!nameToToolBarMap.empty()){
            for(auto& kv : nameToToolBarMap){
                auto toolBar = kv.second;
                if(toolBar->isVisibleByDefault()){
                    newToolBarsToShow.push_back(kv.second);
                }
            }
            if(!newToolBarsToShow.empty()){
                layoutToolBars();
            }
        }
    }

}


void ToolBarArea::resetLayout(Mapping* archive)
{
    impl->resetLayout(archive);
}


void ToolBarArea::Impl::resetLayout(Mapping* archive)
{
    if(DEBUG_MODE){
        cout << "ToolBarArea::Impl::resetLayout()" << endl;
    }
    newToolBarsToShow.clear();
    for(auto& toolBar : registeredToolBars){
        toolBar->hide();
        if(toolBar->isVisibleByDefault()){
            newToolBarsToShow.push_back(toolBar);
        }
    } 
    toolBarRows.clear();
    layoutToolBars();
    
    self->removeLayout(archive);
}


void ToolBarArea::removeLayout(Mapping* archive)
{
    archive->remove("layoutOfToolBars");
}


void ToolBarArea::storeLayout(Mapping* archive)
{
    impl->storeLayout(archive);
}


void ToolBarArea::Impl::storeLayout(Mapping* archive)
{
    auto layoutOfToolBars = archive->createMapping("layoutOfToolBars");
    int numVisibleToolBars = 0;

    auto rows = layoutOfToolBars->createListing("rows");
    for(size_t i=0; i < toolBarRows.size(); ++i){
        auto& toolBars = toolBarRows[i]->toolBars;
        if(!toolBars.empty()){
            auto bars = new Listing;
            for(auto& toolBar : toolBars){
                auto state = new Mapping;
                state->setFlowStyle(true);
                state->write("name", toolBar->objectName().toStdString(), DOUBLE_QUOTED);
                state->write("x", toolBar->desiredX);
                state->write("priority", toolBar->layoutPriority);
                bars->append(state);
                ++numVisibleToolBars;
            }
            rows->append(bars);
        }
    }

    if(numVisibleToolBars > 0){
        ListingPtr hidden = new Listing;
        for(auto& toolBar : registeredToolBars){
            if(toolBar->isVisibleByDefault() && !toolBar->isVisible()){
                hidden->append(toolBar->objectName().toStdString(), DOUBLE_QUOTED);
            }
        }
        if(!hidden->empty()){
            layoutOfToolBars->insert("hidden", hidden);
        }
    }
}


void ToolBarArea::dragToolBar(ToolBar* toolBar, const QPoint& globalPos)
{
    impl->dragToolBar(toolBar, globalPos);
}


void ToolBarArea::Impl::dragToolBar(ToolBar* toolBar, const QPoint& globalPos)
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
    draggedToolBar = nullptr;
}


void ToolBarArea::Impl::setNewToolBars()
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


void ToolBarArea::Impl::expandStrechableBars
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


void ToolBarArea::Impl::setNewToolBar(ToolBar* toolBar, vector<int>& numStrechablesOfRow)
{
    if(DEBUG_MODE){
        cout << "ToolBarArea::Impl::setNewToolBar()" << endl;
    }
    
    if(toolBar){

        ToolBarRowPtr toolBarRow;
        int rowIndex = -1;
        int width = toolBar->minimumSizeHint().width();

        if(!toolBar->isPlacedOnNewRowByDefault()){
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


void ToolBarArea::Impl::layoutToolBars()
{
    if(DEBUG_MODE){
        cout << "ToolBarArea::Impl::layoutToolBars()" << endl;
    }

    if(!newToolBarsToShow.empty()){
        setNewToolBars();
    }

    if(DEBUG_MODE){
        cout << "actually do ToolBarArea::Impl::layoutToolBars()" << endl;
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


void ToolBarArea::Impl::layoutToolBarRow(ToolBarRowPtr toolBarRow, int& io_rowTop, bool isNewBottomRow)
{
    int rowHeight = 0;

    bool draggedToolBarExists = false;
    auto& toolBars = toolBarRow->toolBars;

    // calculate the row height and remove the dragged tool bar if it is originally in this row
    auto p = toolBars.begin();
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


void ToolBarArea::Impl::layoutToolBarRowWithDraggedToolBar(vector<ToolBar*>& toolBars, int rowTop, int rowHeight)
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
int ToolBarArea::Impl::normalizeLayoutPriorities(vector<ToolBar*>& toolBars)
{
    int maxPriority = 0;

    if(!toolBars.empty()){

        vector<ToolBar*> sortedToolBars(toolBars);
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


void ToolBarArea::Impl::layoutToolBarRowPart
(vector<ToolBar*>& toolBars, int rowTop, int rowHeight, int partLeft, int partRight)
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

    vector<ToolBar*> leftPartToolBars;
    int pivotAreaLeft = partLeft;
    for(int i=0; i < pivotIndex; ++i){
        leftPartToolBars.push_back(toolBars[i]);
        pivotAreaLeft += toolBars[i]->minimumSizeHint().width();
    }

    vector<ToolBar*> rightPartToolBars;
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
