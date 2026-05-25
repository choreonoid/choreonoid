#include "PyQString.h"
#include "PyQtSignal.h"
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <QAbstractItemView>
#include <QItemSelectionModel>
#include <QHeaderView>
#include <QTableView>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <vector>

using namespace std;
namespace nb = nanobind;

namespace cnoid {

void exportPyQtModelViewClasses(nb::module_ m)
{
    nb::class_<QAbstractItemView, QAbstractScrollArea>
        qAbstractItemView(m, "QAbstractItemView");

    nb::enum_<QAbstractItemView::ScrollMode>(qAbstractItemView, "ScrollMode")
        .value("ScrollPerItem", QAbstractItemView::ScrollPerItem)
        .value("ScrollPerPixel", QAbstractItemView::ScrollPerPixel)
        .export_values();

    nb::enum_<QAbstractItemView::ScrollHint>(qAbstractItemView, "ScrollHint")
        .value("EnsureVisible", QAbstractItemView::EnsureVisible)
        .value("PositionAtTop", QAbstractItemView::PositionAtTop)
        .value("PositionAtBottom", QAbstractItemView::PositionAtBottom)
        .value("PositionAtCenter", QAbstractItemView::PositionAtCenter)
        .export_values();

    nb::enum_<QAbstractItemView::SelectionBehavior>(qAbstractItemView, "SelectionBehavior")
        .value("SelectItems", QAbstractItemView::SelectItems)
        .value("SelectRows", QAbstractItemView::SelectRows)
        .value("SelectColumns", QAbstractItemView::SelectColumns)
        .export_values();

    nb::enum_<QAbstractItemView::SelectionMode>(qAbstractItemView, "SelectionMode")
        .value("SingleSelection", QAbstractItemView::SingleSelection)
        .value("ContiguousSelection", QAbstractItemView::ContiguousSelection)
        .value("ExtendedSelection", QAbstractItemView::ExtendedSelection)
        .value("MultiSelection", QAbstractItemView::MultiSelection)
        .value("NoSelection", QAbstractItemView::NoSelection)
        .export_values();

    qAbstractItemView
        .def("alternatingRowColors", &QAbstractItemView::alternatingRowColors)
        .def("autoScrollMargin", &QAbstractItemView::autoScrollMargin)
        .def("closePersistentEditor", &QAbstractItemView::closePersistentEditor)
        .def("currentIndex", &QAbstractItemView::currentIndex)
        .def("defaultDropAction", &QAbstractItemView::defaultDropAction)
        .def("dragDropMode", &QAbstractItemView::dragDropMode)
        .def("dragDropOverwriteMode", &QAbstractItemView::dragDropOverwriteMode)
        .def("dragEnabled", &QAbstractItemView::dragEnabled)
        .def("editTriggers", &QAbstractItemView::editTriggers)
        .def("hasAutoScroll", &QAbstractItemView::hasAutoScroll)
        .def("horizontalScrollMode", &QAbstractItemView::horizontalScrollMode)
        .def("iconSize", &QAbstractItemView::iconSize)
        .def("indexAt", &QAbstractItemView::indexAt)
        .def("indexWidget", &QAbstractItemView::indexWidget, nb::rv_policy::reference)
        .def("itemDelegate", (QAbstractItemDelegate*(QAbstractItemView::*)()const) &QAbstractItemView::itemDelegate, nb::rv_policy::reference)

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        .def("itemDelegateForIndex", (QAbstractItemDelegate*(QAbstractItemView::*)(const QModelIndex&)const) &QAbstractItemView::itemDelegateForIndex, nb::rv_policy::reference)
#else
        .def("itemDelegate", (QAbstractItemDelegate*(QAbstractItemView::*)(const QModelIndex&)const) &QAbstractItemView::itemDelegate, nb::rv_policy::reference)
#endif
        .def("itemDelegateForColumn", &QAbstractItemView::itemDelegateForColumn, nb::rv_policy::reference)
        .def("itemDelegateForRow", &QAbstractItemView::itemDelegateForRow, nb::rv_policy::reference)
        .def("keyboardSearch", &QAbstractItemView::keyboardSearch)
        .def("model", &QAbstractItemView::model, nb::rv_policy::reference)
        .def("openPersistentEditor", &QAbstractItemView::openPersistentEditor)
        .def("rootIndex", &QAbstractItemView::rootIndex)
        .def("scrollTo", &QAbstractItemView::scrollTo, nb::arg("index"), nb::arg("hint") = QAbstractItemView::EnsureVisible)
        .def("selectionBehavior", &QAbstractItemView::selectionBehavior)
        .def("selectionMode", &QAbstractItemView::selectionMode)
        .def("selectionModel", &QAbstractItemView::selectionModel, nb::rv_policy::reference)
        .def("setAlternatingRowColors", &QAbstractItemView::setAlternatingRowColors)
        .def("setAutoScroll", &QAbstractItemView::setAutoScroll)
        .def("setAutoScrollMargin", &QAbstractItemView::setAutoScrollMargin)
        .def("setDefaultDropAction", &QAbstractItemView::setDefaultDropAction)
        .def("setDragDropMode", &QAbstractItemView::setDragDropMode)
        .def("setDragDropOverwriteMode", &QAbstractItemView::setDragDropOverwriteMode)
        .def("setDragEnabled", &QAbstractItemView::setDragEnabled)
        .def("setDropIndicatorShown", &QAbstractItemView::setDropIndicatorShown)
        .def("setEditTriggers", &QAbstractItemView::setEditTriggers)
        .def("setHorizontalScrollMode", &QAbstractItemView::setHorizontalScrollMode)
        .def("setIconSize", &QAbstractItemView::setIconSize)
        .def("setIndexWidget", &QAbstractItemView::setIndexWidget)
        .def("setItemDelegate", &QAbstractItemView::setItemDelegate)
        .def("setItemDelegateForColumn", &QAbstractItemView::setItemDelegateForColumn)
        .def("setItemDelegateForRow", &QAbstractItemView::setItemDelegateForRow)
        .def("setModel", &QAbstractItemView::setModel)
        .def("setSelectionBehavior", &QAbstractItemView::setSelectionBehavior)
        .def("setSelectionMode", &QAbstractItemView::setSelectionMode)
        .def("setSelectionModel", &QAbstractItemView::setSelectionModel)
        .def("setTabKeyNavigation", &QAbstractItemView::setTabKeyNavigation)
        .def("setTextElideMode", &QAbstractItemView::setTextElideMode)
        .def("setVerticalScrollMode", &QAbstractItemView::setVerticalScrollMode)
        .def("showDropIndicator", &QAbstractItemView::showDropIndicator)
        .def("sizeHintForColumn", &QAbstractItemView::sizeHintForColumn)
        .def("sizeHintForIndex", &QAbstractItemView::sizeHintForIndex)
        .def("sizeHintForRow", &QAbstractItemView::sizeHintForRow)
        .def("tabKeyNavigation", &QAbstractItemView::tabKeyNavigation)
        .def("textElideMode", &QAbstractItemView::textElideMode)
        .def("verticalScrollMode", &QAbstractItemView::verticalScrollMode)
        .def("visualRect", &QAbstractItemView::visualRect)
        .def("clearSelection", &QAbstractItemView::clearSelection)
        .def("scrollToBottom", &QAbstractItemView::scrollToBottom)
        .def("scrollToTop", &QAbstractItemView::scrollToTop)
        .def("selectAll", &QAbstractItemView::selectAll)
        ;

    nb::class_<QItemSelectionModel, QObject>
        qItemSelectionModel(m, "QItemSelectionModel");

    nb::enum_<QItemSelectionModel::SelectionFlag>(qItemSelectionModel, "SelectionFlag", nb::is_arithmetic())
        .value("NoUpdate", QItemSelectionModel::NoUpdate)
        .value("Clear", QItemSelectionModel::Clear)
        .value("Select", QItemSelectionModel::Select)
        .value("Deselect", QItemSelectionModel::Deselect)
        .value("Toggle", QItemSelectionModel::Toggle)
        .value("Current", QItemSelectionModel::Current)
        .value("Rows", QItemSelectionModel::Rows)
        .value("Columns", QItemSelectionModel::Columns)
        .value("SelectCurrent", QItemSelectionModel::SelectCurrent)
        .value("ToggleCurrent", QItemSelectionModel::ToggleCurrent)
        .value("ClearAndSelect", QItemSelectionModel::ClearAndSelect)
        .export_values();

    nb::class_<QHeaderView, QAbstractItemView>
               qHeaderView(m, "QHeaderView");

    nb::enum_<QHeaderView::ResizeMode>(qHeaderView, "ResizeMode")
        .value("Interactive", QHeaderView::Interactive)
        .value("Fixed", QHeaderView::Fixed)
        .value("Stretch", QHeaderView::Stretch)
        .value("ResizeToContents", QHeaderView::ResizeToContents)
        .export_values()
        ;

    qHeaderView
        .def("count", &QHeaderView::count)
        .def("defaultAlignment", &QHeaderView::defaultAlignment)
        .def("defaultSectionSize", &QHeaderView::defaultSectionSize)
        .def("hiddenSectionCount", &QHeaderView::hiddenSectionCount)
        .def("hideSection", &QHeaderView::hideSection)
        .def("highlightSections", &QHeaderView::highlightSections)
        .def("isSectionHidden", &QHeaderView::isSectionHidden)
        .def("isSortIndicatorShown", &QHeaderView::isSortIndicatorShown)
        .def("length", &QHeaderView::length)
        .def("logicalIndex", &QHeaderView::logicalIndex)
        .def("logicalIndexAt", (int (QHeaderView::*)(int)const) &QHeaderView::logicalIndexAt)
        .def("logicalIndexAt", (int (QHeaderView::*)(int, int)const) &QHeaderView::logicalIndexAt)
        .def("logicalIndexAt", (int (QHeaderView::*)(const QPoint&)const) &QHeaderView::logicalIndexAt)
        .def("maximumSectionSize", &QHeaderView::maximumSectionSize)
        .def("minimumSectionSize", &QHeaderView::minimumSectionSize)
        .def("moveSection", &QHeaderView::moveSection)
        .def("offset", &QHeaderView::offset)
        .def("orientation", &QHeaderView::orientation)
        .def("resetDefaultSectionSize", &QHeaderView::resetDefaultSectionSize)
        .def("resizeContentsPrecision", &QHeaderView::resizeContentsPrecision)
        .def("resizeSection", &QHeaderView::resizeSection)
        .def("resizeSections", (void(QHeaderView::*)(QHeaderView::ResizeMode)) &QHeaderView::resizeSections)
        .def("sectionPosition", &QHeaderView::sectionPosition)
        .def("sectionResizeMode", &QHeaderView::sectionResizeMode)
        .def("sectionSize", &QHeaderView::sectionSize)
        .def("sectionSizeHint", &QHeaderView::sectionSizeHint)
        .def("sectionViewportPosition", &QHeaderView::sectionViewportPosition)
        .def_prop_ro("sectionsClickable", &QHeaderView::sectionsClickable)
        .def_prop_ro("sectionsHidden", &QHeaderView::sectionsHidden)
        .def_prop_ro("sectionsMovable", &QHeaderView::sectionsMovable)
        .def_prop_ro("sectionsMoved", &QHeaderView::sectionsMoved)
        .def("setCascadingSectionResizes", &QHeaderView::setCascadingSectionResizes)
        .def("setDefaultAlignment", &QHeaderView::setDefaultAlignment)
        .def("setDefaultSectionSize", &QHeaderView::setDefaultSectionSize)
        .def("setHighlightSections", &QHeaderView::setHighlightSections)
        .def("setMaximumSectionSize", &QHeaderView:: setMaximumSectionSize)
        .def("setMinimumSectionSize", &QHeaderView::setMinimumSectionSize)
        .def("setResizeContentsPrecision", &QHeaderView::setResizeContentsPrecision)
        .def("setSectionHidden", &QHeaderView::setSectionHidden)
        .def("setSectionResizeMode", (void (QHeaderView::*)(QHeaderView::ResizeMode)) &QHeaderView::setSectionResizeMode)
        .def("setSectionResizeMode", (void (QHeaderView::*)(int, QHeaderView::ResizeMode)) &QHeaderView::setSectionResizeMode)
        .def("setSectionsClickable", &QHeaderView::setSectionsClickable)
        .def("setSectionsMovable", &QHeaderView::setSectionsMovable)
        .def("setSortIndicator", &QHeaderView::setSortIndicator)
        .def("setSortIndicatorShown", &QHeaderView::setSortIndicatorShown)
        .def("setStretchLastSection", &QHeaderView::setStretchLastSection)
        .def("showSection", &QHeaderView::showSection)
        .def("sortIndicatorOrder", &QHeaderView::sortIndicatorOrder)
        .def("sortIndicatorSection", &QHeaderView::sortIndicatorSection)
        .def("stretchLastSection", &QHeaderView::stretchLastSection)
        .def("stretchSectionCount", &QHeaderView::stretchSectionCount)
        .def("swapSections", &QHeaderView::swapSections)
        .def("visualIndex", &QHeaderView::visualIndex)
        .def("visualIndexAt", &QHeaderView::visualIndexAt)
        ;

    nb::class_<QTableView, QAbstractItemView>(m, "QTableView")
        .def(nb::init<>())
        .def("clearSpans", &QTableView::clearSpans)
        .def("columnAt", &QTableView::columnAt)
        .def("columnSpan", &QTableView::columnSpan)
        .def("columnViewportPosition", &QTableView::columnViewportPosition)
        .def("columnWidth", &QTableView::columnWidth)
        .def("gridStyle", &QTableView::gridStyle)
        .def("horizontalHeader", &QTableView::horizontalHeader, nb::rv_policy::reference)
        .def("isColumnHidden", &QTableView::isColumnHidden)
        .def("isCornerButtonEnabled", &QTableView::isCornerButtonEnabled)
        .def("isRowHidden", &QTableView::isRowHidden)
        .def("isSortingEnabled", &QTableView::isSortingEnabled)
        .def("rowAt", &QTableView::rowAt)
        .def("rowHeight", &QTableView::rowHeight)
        .def("rowSpan", &QTableView::rowSpan)
        .def("rowViewportPosition", &QTableView::rowViewportPosition)
        .def("setColumnHidden", &QTableView::setColumnHidden)
        .def("setColumnWidth", &QTableView::setColumnWidth)
        .def("setCornerButtonEnabled", &QTableView::setCornerButtonEnabled)
        .def("setGridStyle", &QTableView::setGridStyle)
        .def("setHorizontalHeader", &QTableView::setHorizontalHeader)
        .def("setRowHeight", &QTableView::setRowHeight)
        .def("setRowHidden", &QTableView::setRowHidden)
        .def("setSortingEnabled", &QTableView::setSortingEnabled)
        .def("setSpan", &QTableView::setSpan)
        .def("setVerticalHeader", &QTableView::setVerticalHeader)
        .def("setWordWrap", &QTableView::setWordWrap)
        .def("showGrid", &QTableView::showGrid)
        .def("sortByColumn", (void(QTableView::*)(int,Qt::SortOrder)) &QTableView::sortByColumn)
        .def("verticalHeader", &QTableView::verticalHeader, nb::rv_policy::reference)
        .def("wordWrap", &QTableView::wordWrap)
        .def("hideColumn", &QTableView::hideColumn)
        .def("hideRow", &QTableView::hideRow)
        .def("resizeColumnToContents", &QTableView::resizeColumnToContents)
        .def("resizeColumnsToContents", &QTableView::resizeColumnsToContents)
        .def("resizeRowToContents", &QTableView::resizeRowToContents)
        .def("resizeRowsToContents", &QTableView::resizeRowsToContents)
        .def("selectColumn", &QTableView::selectColumn)
        .def("selectRow", &QTableView::selectRow)
        .def("setShowGrid", &QTableView::setShowGrid)
        .def("showColumn", &QTableView::showColumn)
        .def("showRow", &QTableView::showRow)
        ;

    nb::class_<QTableWidget, QTableView> qTableWidget(m, "QTableWidget");

    typedef cnoid::QtSignal<void(QTableWidget::*)(), void()> QTableWidgetSignal;
    cnoid::PyQtSignal<QTableWidgetSignal>(qTableWidget, "Signal");

    typedef cnoid::QtSignal<void(QTableWidget::*)(int,int), void(int,int)> QTableWidgetInt2Signal;
    cnoid::PyQtSignal<QTableWidgetInt2Signal>(qTableWidget, "Int2Signal");

    typedef cnoid::QtSignal<void(QTableWidget::*)(int,int,int,int), void(int,int,int,int)> QTableWidgetInt4Signal;
    cnoid::PyQtSignal<QTableWidgetInt4Signal>(qTableWidget, "Int4Signal");

    typedef cnoid::QtSignal<void(QTableWidget::*)(QTableWidgetItem*), void(QTableWidgetItem*)> QTableWidgetItemSignal;
    cnoid::PyQtSignal<QTableWidgetItemSignal>(qTableWidget, "ItemSignal");

    typedef cnoid::QtSignal<void(QTableWidget::*)(QTableWidgetItem*,QTableWidgetItem*), void(QTableWidgetItem*,QTableWidgetItem*)> QTableWidgetItem2Signal;
    cnoid::PyQtSignal<QTableWidgetItem2Signal>(qTableWidget, "Item2Signal");

    qTableWidget
        .def(nb::init<>())
        .def("cellWidget", &QTableWidget::cellWidget, nb::rv_policy::reference)
        .def("closePersistentEditor", &QTableWidget::closePersistentEditor)
        .def("column", &QTableWidget::column)
        .def("columnCount", &QTableWidget::columnCount)
        .def("currentColumn", &QTableWidget::currentColumn)
        .def("currentItem", &QTableWidget::currentItem, nb::rv_policy::reference)
        .def("currentRow", &QTableWidget::currentRow)
        .def("editItem", &QTableWidget::editItem)
        .def("findItems",
             [](const QTableWidget& self, const std::string& text, Qt::MatchFlags flags){
                 auto qitems = self.findItems(text.c_str(), flags);
                 vector<QTableWidgetItem*> items;
                 items.reserve(qitems.count());
                 for(auto& item : qitems){
                     items.push_back(item);
                 }
                 return items;
             }, nb::rv_policy::reference)
        .def("horizontalHeaderItem", &QTableWidget::horizontalHeaderItem, nb::rv_policy::reference)
        .def("item", &QTableWidget::item, nb::rv_policy::reference)
        .def("itemAt", (QTableWidgetItem*(QTableWidget::*)(const QPoint&)const) &QTableWidget::itemAt, nb::rv_policy::reference)
        .def("itemAt", (QTableWidgetItem*(QTableWidget::*)(int, int)const) &QTableWidget::itemAt, nb::rv_policy::reference)
        .def("itemPrototype", &QTableWidget::itemPrototype, nb::rv_policy::reference)
        .def("openPersistentEditor", &QTableWidget::openPersistentEditor)
        .def("removeCellWidget", &QTableWidget::removeCellWidget)
        .def("row", &QTableWidget::row)
        .def("rowCount", &QTableWidget::rowCount)
        .def(
            "selectedItems",
            [](QTableWidget& self){
                auto src = self.selectedItems();
                std::vector<QTableWidgetItem*> items;
                items.reserve(src.size());
                std::copy(src.begin(), src.end(), std::back_inserter(items));
                return items;
            }, nb::rv_policy::reference)

        .def("setCellWidget", &QTableWidget::setCellWidget)
        .def("setColumnCount", &QTableWidget::setColumnCount)
        .def("setCurrentCell", (void(QTableWidget::*)(int, int)) &QTableWidget::setCurrentCell)
        .def("setCurrentCell",
             (void(QTableWidget::*)(int, int, QItemSelectionModel::SelectionFlags))
             &QTableWidget::setCurrentCell)
        .def("setCurrentItem", (void(QTableWidget::*)(QTableWidgetItem*)) &QTableWidget::setCurrentItem)
        .def("setCurrentItem",
             (void(QTableWidget::*)(QTableWidgetItem*, QItemSelectionModel::SelectionFlags command))
             &QTableWidget::setCurrentItem)
        .def("setHorizontalHeaderItem", &QTableWidget::setHorizontalHeaderItem)
        .def("setHorizontalHeaderLabels",
             [](QTableWidget& self, const std::vector<std::string>& labels){
                 QStringList list;
                 list.reserve(labels.size());
                 for(auto& label : labels){
                     list.append(label.c_str());
                 }
                 self.setHorizontalHeaderLabels(list);
             })
        .def("setItem", &QTableWidget::setItem)
        .def("setItemPrototype", &QTableWidget::setItemPrototype)
        .def("setRangeSelected", &QTableWidget::setRangeSelected)
        .def("setRowCount", &QTableWidget::setRowCount)
        .def("setVerticalHeaderItem", &QTableWidget::setVerticalHeaderItem)
        .def("setVerticalHeaderLabels",
             [](QTableWidget& self, const std::vector<std::string>& labels){
                 QStringList list;
                 list.reserve(labels.size());
                 for(auto& label : labels){
                     list.append(label.c_str());
                 }
                 self.setVerticalHeaderLabels(list);
             })
        .def("sortItems",
             &QTableWidget::sortItems, nb::arg("column"), nb::arg("order") = Qt::AscendingOrder)
        .def("takeHorizontalHeaderItem", &QTableWidget::takeHorizontalHeaderItem, nb::rv_policy::reference)
        .def("takeItem", &QTableWidget::takeItem, nb::rv_policy::reference)
        .def("takeVerticalHeaderItem", &QTableWidget::takeVerticalHeaderItem, nb::rv_policy::reference)
        .def("verticalHeaderItem", &QTableWidget::verticalHeaderItem, nb::rv_policy::reference)
        .def("visualColumn", &QTableWidget::visualColumn)
        .def("visualItemRect", &QTableWidget::visualItemRect)
        .def("visualRow", &QTableWidget::visualRow)

        .def("clear", &QTableWidget::clear)
        .def("clearContents", &QTableWidget::clearContents)
        .def("insertColumn", &QTableWidget::insertColumn)
        .def("insertRow", &QTableWidget::insertRow)
        .def("removeColumn", &QTableWidget::removeColumn)
        .def("removeRow", &QTableWidget::removeRow)
        .def("scrollToItem", &QTableWidget::scrollToItem, nb::arg("item"), nb::arg("hint") = QAbstractItemView::EnsureVisible)

        .def_prop_ro("cellActivated", [](QTableWidget* self){ return QTableWidgetInt2Signal(self, &QTableWidget::cellActivated); })
        .def_prop_ro("cellChanged", [](QTableWidget* self){ return QTableWidgetInt2Signal(self, &QTableWidget::cellChanged); })
        .def_prop_ro("cellClicked", [](QTableWidget* self){ return QTableWidgetInt2Signal(self, &QTableWidget::cellClicked); })
        .def_prop_ro("cellDoubleClicked", [](QTableWidget* self){ return QTableWidgetInt2Signal(self, &QTableWidget::cellDoubleClicked); })
        .def_prop_ro("cellEntered", [](QTableWidget* self){ return QTableWidgetInt2Signal(self, &QTableWidget::cellEntered); })
        .def_prop_ro("cellPressed", [](QTableWidget* self){ return QTableWidgetInt2Signal(self, &QTableWidget::cellPressed); })
        .def_prop_ro("currentCellChanged", [](QTableWidget* self){ return QTableWidgetInt4Signal(self, &QTableWidget::currentCellChanged); })
        .def_prop_ro("currentItemChanged", [](QTableWidget* self){ return QTableWidgetItem2Signal(self, &QTableWidget::currentItemChanged); })
        .def_prop_ro("itemActivated", [](QTableWidget* self){ return QTableWidgetItemSignal(self, &QTableWidget::itemActivated); })
        .def_prop_ro("itemChanged", [](QTableWidget* self){ return QTableWidgetItemSignal(self, &QTableWidget::itemChanged); })
        .def_prop_ro("itemClicked", [](QTableWidget* self){ return QTableWidgetItemSignal(self, &QTableWidget::itemClicked); })
        .def_prop_ro("itemDoubleClicked", [](QTableWidget* self){ return QTableWidgetItemSignal(self, &QTableWidget::itemDoubleClicked); })
        .def_prop_ro("itemEntered", [](QTableWidget* self){ return QTableWidgetItemSignal(self, &QTableWidget::itemEntered); })
        .def_prop_ro("itemPressed", [](QTableWidget* self){ return QTableWidgetItemSignal(self, &QTableWidget::itemPressed); })
        .def_prop_ro("itemSelectionChanged", [](QTableWidget* self){ return QTableWidgetSignal(self, &QTableWidget::itemSelectionChanged); })
        ;

    // QTableWidgetItem is owned by its QTableWidget on the Qt side, so it is
    // registered without a holder and not deleted by nanobind.
    nb::class_<QTableWidgetItem> qTableWidgetItem(m, "QTableWidgetItem");

    nb::enum_<QTableWidgetItem::ItemType>(qTableWidgetItem, "ItemType")
        .value("Type", QTableWidgetItem::Type)
        .value("UserType", QTableWidgetItem::UserType)
        .export_values();

    qTableWidgetItem
        .def(nb::init<int>(), nb::arg("type") = QTableWidgetItem::Type)
        .def(nb::init<const QString&, int>(), nb::arg("text"), nb::arg("type") = QTableWidgetItem::Type)
        .def("background", &QTableWidgetItem::background)
        .def("checkState", &QTableWidgetItem::checkState)
        .def("clone", &QTableWidgetItem::clone, nb::rv_policy::reference)
        .def("column", &QTableWidgetItem::column)
        .def("data", &QTableWidgetItem::data)
        .def("flags", &QTableWidgetItem::flags)
        .def("font", &QTableWidgetItem::font)
        .def("foreground", &QTableWidgetItem::foreground)
        .def("icon", &QTableWidgetItem::icon)
        .def("isSelected", &QTableWidgetItem::isSelected)
        .def("row", &QTableWidgetItem::row)
        .def("setBackground", &QTableWidgetItem::setBackground)
        .def("setCheckState", &QTableWidgetItem::setCheckState)
        .def("setData", &QTableWidgetItem::setData)
        .def("setFlags", &QTableWidgetItem::setFlags)
        .def("setFont", &QTableWidgetItem::setFont)
        .def("setForeground", &QTableWidgetItem::setForeground)
        .def("setIcon", &QTableWidgetItem::setIcon)
        .def("setSelected", &QTableWidgetItem::setSelected)
        .def("setSizeHint", &QTableWidgetItem::setSizeHint)
        .def("setStatusTip", &QTableWidgetItem::setStatusTip)
        .def("setText", &QTableWidgetItem::setText)

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        .def("setTextAlignment", (void(QTableWidgetItem::*)(Qt::Alignment)) &QTableWidgetItem::setTextAlignment)
#else
        .def("setTextAlignment", (void(QTableWidgetItem::*)(int)) &QTableWidgetItem::setTextAlignment)
#endif
        .def("setToolTip", &QTableWidgetItem::setToolTip)
        .def("setWhatsThis", &QTableWidgetItem::setWhatsThis)
        .def("sizeHint", &QTableWidgetItem::sizeHint)
        .def("statusTip", &QTableWidgetItem::statusTip)
        .def("tableWidget", &QTableWidgetItem::tableWidget, nb::rv_policy::reference)
        .def("text", &QTableWidgetItem::text)
        .def("textAlignment", &QTableWidgetItem::textAlignment)
        .def("toolTip", &QTableWidgetItem::toolTip)
        .def("type", &QTableWidgetItem::type)
        .def("whatsThis", &QTableWidgetItem::whatsThis)
        ;
}

}
