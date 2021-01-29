#include "PyQObjectHolder.h"
#include "PyQString.h"
#include "PyQtSignal.h"
#include <QAbstractItemView>
#include <QItemSelectionModel>
#include <QHeaderView>
#include <QTableView>
#include <QTableWidget>
#include <QTableWidgetItem>

namespace py = pybind11;

namespace cnoid {

void exportPyQtGuiModelViewClasses(py::module m)
{
    py::class_<QAbstractItemView, PyQObjectHolder<QAbstractItemView>, QAbstractScrollArea>
        qAbstractItemView(m, "QAbstractItemView");

    py::enum_<QAbstractItemView::ScrollMode>(qAbstractItemView, "ScrollMode")
        .value("ScrollPerItem", QAbstractItemView::ScrollPerItem)
        .value("ScrollPerPixel", QAbstractItemView::ScrollPerPixel)
        .export_values();
    
    py::enum_<QAbstractItemView::ScrollHint>(qAbstractItemView, "ScrollHint")
        .value("QAbstractItemView::EnsureVisible", QAbstractItemView::EnsureVisible)
        .value("QAbstractItemView::PositionAtTop", QAbstractItemView::PositionAtTop)
        .value("QAbstractItemView::PositionAtBottom", QAbstractItemView::PositionAtBottom)
        .value("QAbstractItemView::PositionAtCenter", QAbstractItemView::PositionAtCenter)
        .export_values();
    
    py::enum_<QAbstractItemView::SelectionBehavior>(qAbstractItemView, "SelectionBehavior")
        .value("SelectItems", QAbstractItemView::SelectItems)
        .value("SelectRows", QAbstractItemView::SelectRows)
        .value("SelectColumns", QAbstractItemView::SelectColumns)
        .export_values();
    
    py::enum_<QAbstractItemView::SelectionMode>(qAbstractItemView, "SelectionMode")
        .value("SingleSelection", QAbstractItemView::SingleSelection)
        .value("ContiguousSelection", QAbstractItemView::ContiguousSelection)
        .value("ExtendedSelection", QAbstractItemView::ExtendedSelection)
        .value("MultiSelection", QAbstractItemView::MultiSelection)
        .value("NoSelection", QAbstractItemView::NoSelection)
        .export_values();
    
    qAbstractItemView
        .def("hasAutoScroll", &QAbstractItemView::hasAutoScroll)
        .def("setAutoScroll", &QAbstractItemView::setAutoScroll)
        .def("horizontalScrollMode", &QAbstractItemView::horizontalScrollMode)
        .def("setHorizontalScrollMode", &QAbstractItemView::setHorizontalScrollMode)
	.def("verticalScrollMode", &QAbstractItemView::verticalScrollMode)
        .def("setVerticalScrollMode", &QAbstractItemView::setVerticalScrollMode)
        .def("scrollToBottom", &QAbstractItemView::scrollToBottom)
        .def("scrollToTop", &QAbstractItemView::scrollToTop)
        .def("selectionMode", &QAbstractItemView::selectionMode)
        .def("selectionModel", &QAbstractItemView::selectionModel)
        .def("setSelectionBehavior", &QAbstractItemView::setSelectionBehavior)
        .def("setSelectionMode", &QAbstractItemView::setSelectionMode)
        .def("clearSelection", &QAbstractItemView::clearSelection)
        .def("selectAll", &QAbstractItemView::selectAll)
        .def("setAlternatingRowColors", &QAbstractItemView::setAlternatingRowColors)
        ;

    py::class_<QItemSelectionModel, PyQObjectHolder<QItemSelectionModel>, QObject>
        qItemSelectionModel(m, "QItemSelectionModel");

    py::enum_<QItemSelectionModel::SelectionFlag>(qItemSelectionModel, "SelectionFlag")
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

    py::class_<QHeaderView, PyQObjectHolder<QHeaderView>, QAbstractItemView>
               qHeaderView(m, "QHeaderView");

    py::enum_<QHeaderView::ResizeMode>(qHeaderView, "ResizeMode")
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
        .def_property_readonly("sectionsClickable", &QHeaderView::sectionsClickable)
        .def_property_readonly("sectionsHidden", &QHeaderView::sectionsHidden)
        .def_property_readonly("sectionsMovable", &QHeaderView::sectionsMovable)
        .def_property_readonly("sectionsMoved", &QHeaderView::sectionsMoved)
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

    py::class_<QTableView, PyQObjectHolder<QTableView>, QAbstractItemView>(m, "QTableView")
        .def(py::init<>())
        .def("horizontalHeader", &QTableView::horizontalHeader)
        .def("verticalHeader", &QTableView::verticalHeader)
        .def("isSortingEnabled", &QTableView::isSortingEnabled)
        .def("columnWidth", &QTableView::columnWidth)
        .def("setWordWrap", &QTableView::setWordWrap)
        .def("wordWrap", &QTableView::wordWrap)
        ;

    py::class_<QTableWidget, PyQObjectHolder<QTableWidget>, QTableView>(m, "QTableWidget")
        .def(py::init<>())
        .def("getCellWidget", &QTableWidget::cellWidget)
        .def("closePersistentEditor", &QTableWidget::closePersistentEditor)
        .def("getColumn", &QTableWidget::column)
        .def("columnCount", &QTableWidget::columnCount)
        .def("currentColumn", &QTableWidget::currentColumn)
        .def("currentItem", &QTableWidget::currentItem)
        .def("currentRow", &QTableWidget::currentRow)
        .def("editItem", &QTableWidget::editItem)
        .def("horizontalHeaderItem", &QTableWidget::horizontalHeaderItem)

        //.def("isPersistentEditorOpen", &QTableWidget::isPersistentEditorOpen)
        
        .def("item", &QTableWidget::item)
        .def("itemAt", (QTableWidgetItem*(QTableWidget::*)(const QPoint&)const) &QTableWidget::itemAt)
        .def("itemAt", (QTableWidgetItem*(QTableWidget::*)(int, int)const) &QTableWidget::itemAt)
        .def("itemPrototype", &QTableWidget::itemPrototype)
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
            })
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
             &QTableWidget::sortItems, py::arg("column"), py::arg("order") = Qt::AscendingOrder)
        .def("takeHorizontalHeaderItem", &QTableWidget::takeHorizontalHeaderItem)
        .def("takeItem", &QTableWidget::takeItem)
        .def("takeVerticalHeaderItem", &QTableWidget::takeVerticalHeaderItem)
        .def("verticalHeaderItem", &QTableWidget::verticalHeaderItem)
        .def("visualColumn",	&QTableWidget::visualColumn)
        .def("visualItemRect", &QTableWidget::visualItemRect)
        .def("visualRow", &QTableWidget::visualRow)
        .def("clear", &QTableWidget::clear)
        .def("clearContents", &QTableWidget::clearContents)
        .def("insertColumn", &QTableWidget::insertColumn)
        .def("insertRow", &QTableWidget::insertRow)
        .def("removeColumn", &QTableWidget::removeColumn)
        .def("removeRow", &QTableWidget::removeRow)
        .def("scrollToItem", &QTableWidget::scrollToItem, py::arg("item"), py::arg("hint") = QAbstractItemView::EnsureVisible)
        ;

    py::class_<QTableWidgetItem, std::unique_ptr<QTableWidgetItem, py::nodelete>>
        qTableWidgetItem(m, "QTableWidgetItem");

    py::enum_<QTableWidgetItem::ItemType>(qTableWidgetItem, "ItemType")
        .value("Type", QTableWidgetItem::Type)
        .value("UserType", QTableWidgetItem::UserType)
        .export_values();

    qTableWidgetItem
        .def(py::init<int>(), py::arg("type") = QTableWidgetItem::Type)
        .def(py::init<const QString&, int>(), py::arg("text"), py::arg("type") = QTableWidgetItem::Type)
        .def("background", &QTableWidgetItem::background)
        .def("checkState", &QTableWidgetItem::checkState)
        .def("clone", &QTableWidgetItem::clone)
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
        .def("setTextAlignment", &QTableWidgetItem::setTextAlignment)
        .def("setToolTip", &QTableWidgetItem::setToolTip)
        .def("setWhatsThis", &QTableWidgetItem::setWhatsThis)
        .def("sizeHint", &QTableWidgetItem::sizeHint)
        .def("statusTip", &QTableWidgetItem::statusTip)
        .def("tableWidget", &QTableWidgetItem::tableWidget)
        .def("text", &QTableWidgetItem::text)
        .def("textAlignment", &QTableWidgetItem::textAlignment)
        .def("toolTip", &QTableWidgetItem::toolTip)
        .def("type", &QTableWidgetItem::type)
        .def("whatsThis", &QTableWidgetItem::whatsThis)
        ;
}

}
