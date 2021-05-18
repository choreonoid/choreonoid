#include "ItemPropertyWidget.h"
#include "PolymorphicItemFunctionSet.h"
#include "PutPropertyFunction.h"
#include "LazyCaller.h"
#include "MainWindow.h"
#include "Buttons.h"
#include "StringListComboBox.h"
#include "MenuManager.h"
#include "FileDialog.h"
#include <cnoid/ConnectionSet>
#include <cnoid/ExecutablePath>
#include <cnoid/UTF8>
#include <cnoid/stdx/variant>
#include <cnoid/stdx/filesystem>
#include <QTableWidget>
#include <QHeaderView>
#include <QBoxLayout>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QStyledItemDelegate>
#include <QItemEditorFactory>
#include <QStandardItemEditorCreator>
#include <QKeyEvent>
#include <QApplication>
#include <fmt/format.h>
#include <regex>
#include <cmath>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

struct Int {
    int value;
    int min, max;
    Int(int v){
        value = v;
    };
    Int(int v, int min, int max) {
        value = v;
        this->min = min;
        this->max = max;
    };
};

struct Double {
    double value;
    int decimals;
    double min, max;
    Double(double v, int d) {
        value = v;
        decimals = d;
    };
    Double(double v, int d, double min, double max) {
        value = v;
        decimals = d;
        this->min = min;
        this->max = max;
    };
};

typedef stdx::variant<bool, Int, Double, string, Selection, FilePathProperty> ValueVariant;

typedef stdx::variant<
    std::function<bool(bool)>,
    std::function<bool(int)>,
    std::function<bool(double)>,
    std::function<bool(const string&)>
    > FunctionVariant;

enum TypeId { TYPE_BOOL, TYPE_INT, TYPE_DOUBLE, TYPE_STRING, TYPE_SELECTION, TYPE_FILEPATH };

class PropertyItem : public QTableWidgetItem
{
public:
    ItemPropertyWidget::Impl* view;
    ValueVariant value;
    FunctionVariant func;
    bool hasValidFunction;

    PropertyItem(ItemPropertyWidget::Impl* view, ValueVariant value);
    PropertyItem(ItemPropertyWidget::Impl* view, ValueVariant value, FunctionVariant func);
    virtual QVariant data(int role) const;
    virtual void setData(int role, const QVariant& qvalue);
};

class CustomizedItemDelegate;

class FilePathEditor : public QWidget
{
    CustomizedItemDelegate* itemDelegate;
    QLineEdit* lineEdit;
    FilePathProperty currentValue;
public:
    FilePathEditor(CustomizedItemDelegate* delegate, QWidget* parent);
    void setValue(const FilePathProperty& value);
    QString value() const;
};


class CustomizedTableWidget : public QTableWidget
{
    bool isResizing;
    int offsetX;
    
public:
    CustomizedTableWidget(QWidget* parent);
    PropertyItem* itemFromIndex(const QModelIndex& index) const;
    bool isPointingBorder(int x);
    virtual void mouseMoveEvent(QMouseEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    virtual void mouseReleaseEvent(QMouseEvent* event) override;
    virtual void resizeEvent(QResizeEvent* event) override;
};

    
class CustomizedItemDelegate : public QStyledItemDelegate
{
public:
    CustomizedTableWidget* tableWidget;
    int decimals;
        
    CustomizedItemDelegate(CustomizedTableWidget* tableWidget);
    virtual QWidget* createEditor(
        QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual void updateEditorGeometry(
        QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual void setEditorData(QWidget* editor, const QModelIndex& index) const override;
    virtual void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;
    virtual QString displayText(const QVariant& value, const QLocale& locale) const override;
    virtual void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    void openFileDialog(FilePathProperty value, FilePathEditor* editor);
};

}

namespace cnoid {

class ItemPropertyWidget::Impl : public PutPropertyFunction
{
public:
    ItemPropertyWidget* self;
    ItemPtr currentItem;
    ScopedConnectionSet itemConnections;
    PolymorphicItemFunctionSet  propertyFunctions;

    int decimals_;
    double dmin;
    double dmax;
    int imin;
    int imax;

    CustomizedTableWidget* tableWidget;
    int fontPointSizeDiff;

    bool isEditingProperty;
    bool updateRequestedDuringPropertyEditing;

    Impl(ItemPropertyWidget* self);

    void setCurrentItem(Item* item);
    void clear();
    void updateProperties(bool isItemChanged = false);
    void addProperty(const std::string& name, PropertyItem* propertyItem);
    void onTargetItemSpecified(Item* item);
    void zoomFontSize(int pointSizeDiff);
    
    // PutPropertyFunction's virtual functions
    virtual PutPropertyFunction& decimals(int d) override;
    virtual PutPropertyFunction& min(double min) override;
    virtual PutPropertyFunction& max(double max) override;
    virtual PutPropertyFunction& min(int min) override;
    virtual PutPropertyFunction& max(int max) override;
    virtual PutPropertyFunction& reset() override;

    virtual void operator()(
        const std::string& name, bool value) override;
    virtual void operator()(
        const std::string& name, bool value, const std::function<bool(bool)>& func) override;
    virtual void operator()(
        const std::string& name, int value) override;
    virtual void operator()(
        const std::string& name, int value, const std::function<bool(int)>& func) override;
    virtual void operator()(
        const std::string& name, double value) override;
    virtual void operator()(
        const std::string& name, double value, const std::function<bool(double)>& func) override;
    virtual void operator()(
        const std::string& name, const std::string& value) override;
    virtual void operator()(
        const std::string& name, const std::string& value,
        const std::function<bool(const std::string&)>& func) override;
    virtual void operator()(
        const std::string& name, const Selection& selection) override;
    virtual void operator()(
        const std::string& name, const Selection& selection,
        const std::function<bool(int which)>& func) override;
    virtual void operator()(
        const std::string& name, const FilePathProperty& filepath) override;
    virtual void operator()(
        const std::string& name, const FilePathProperty& filepath,
        const std::function<bool(const std::string&)>& func) override;
};

}


namespace {

PropertyItem::PropertyItem(ItemPropertyWidget::Impl* view, ValueVariant value)
    : view(view),
      value(value)
{
    setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    hasValidFunction = false;
}


PropertyItem::PropertyItem(ItemPropertyWidget::Impl* view, ValueVariant value, FunctionVariant func)
    : view(view),
      value(value),
      func(func)
{
    setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsEditable);
    hasValidFunction = true;
}


QVariant PropertyItem::data(int role) const
{
    if(role == Qt::DisplayRole || role == Qt::EditRole){
        switch(stdx::get_variant_index(value)){
        case TYPE_BOOL:      return stdx::get<bool>(value);
        case TYPE_INT:       return stdx::get<Int>(value).value;
        case TYPE_DOUBLE:    return stdx::get<Double>(value).value;
        case TYPE_STRING:    return stdx::get<string>(value).c_str();

        case TYPE_SELECTION:
            {
                const Selection& s = stdx::get<Selection>(value);
                if(role == Qt::DisplayRole){
                    return s.selectedLabel();
                } else if(role == Qt::EditRole){
                    QStringList labels;
                    labels << QString::number(s.selectedIndex());
                    for(int i=0; i < s.size(); ++i){
                        labels << s.label(i);
                    }
                    return labels;
                }
            }

        case TYPE_FILEPATH:
            {
                const FilePathProperty& f = stdx::get<FilePathProperty>(value);
                string filename = f.filename();
                if(!f.isFullpathDisplayMode()){
                    filename = toUTF8(
                        filesystem::path(fromUTF8(filename)).filename().string());
                }
                return filename.c_str();
            }
        }
    } else if(role == Qt::ToolTipRole){
        if(stdx::get_variant_index(value) == TYPE_FILEPATH){
            const FilePathProperty& f = stdx::get<FilePathProperty>(value);
            if(!f.isFullpathDisplayMode()){
                string fullpath = f.filename();
                string filename = toUTF8(
                    filesystem::path(fromUTF8(fullpath)).filename().string());
                if(filename != fullpath){
                    return fullpath.c_str();
                }
            }
        }
    }

    return QTableWidgetItem::data(role);
}


void PropertyItem::setData(int role, const QVariant& qvalue)
{
    view->isEditingProperty = true;
    view->updateRequestedDuringPropertyEditing = false;
    
    bool accepted = false;

    if(role == Qt::EditRole){

        switch(qvalue.type()){
                
        case QVariant::Bool:
            accepted = stdx::get<std::function<bool(bool)>>(func)(qvalue.toBool());
            break;
                
        case QVariant::String:
            accepted = stdx::get<std::function<bool(const string&)>>(func)(qvalue.toString().toStdString());
            break;
                
        case QVariant::Int:
            accepted = stdx::get<std::function<bool(int)>>(func)(qvalue.toInt());
            break;
                
        case QVariant::Double:
            accepted = stdx::get<std::function<bool(double)>>(func)(qvalue.toDouble());
            break;
                
        case QVariant::StringList:
        {
            const QStringList& slist = qvalue.toStringList();
            if(!slist.empty()){
                accepted = stdx::get<std::function<bool(int)>>(func)(slist[0].toInt());
            }
        }
        break;
            
        default:
            break;
        }
            
        if(accepted){
            if(!view->updateRequestedDuringPropertyEditing){
                view->currentItem->notifyUpdate();
            }
        }
        if(view->updateRequestedDuringPropertyEditing){
            auto itemPropertyViewImpl = this->view;
            callLater([itemPropertyViewImpl](){ itemPropertyViewImpl->updateProperties(); });
        }
    }

    view->isEditingProperty = false;
}


FilePathEditor::FilePathEditor(CustomizedItemDelegate* delegate, QWidget* parent)
    : QWidget(parent),
      itemDelegate(delegate)
{
    setAutoFillBackground(true);
    auto hbox = new QHBoxLayout;
    setLayout(hbox);
    hbox->setContentsMargins(0, 0, 0, 0);
    hbox->setSpacing(0);
    lineEdit = new QLineEdit(this);
    lineEdit->setFrame(false);
    hbox->addWidget(lineEdit);
    setFocusProxy(lineEdit);
    auto button = new PushButton(QApplication::style()->standardIcon(QStyle::SP_FileDialogStart), "", this);
    button->sigClicked().connect([=](){ itemDelegate->openFileDialog(currentValue, this); });
    hbox->addWidget(button);
}


void FilePathEditor::setValue(const FilePathProperty& value)
{
    currentValue = value;
    lineEdit->setText(value.filename().c_str());
}


QString FilePathEditor::value() const
{
    if(currentValue.baseDirectory().empty()){
        return lineEdit->text();
    } else {
        return toUTF8(
            (filesystem::path(fromUTF8(currentValue.baseDirectory()))
            / fromUTF8(lineEdit->text().toStdString())).string()).c_str();
    }
}


CustomizedTableWidget::CustomizedTableWidget(QWidget* parent)
    : QTableWidget(parent)
{
    setMouseTracking(true);
    isResizing = false;
}
        

PropertyItem* CustomizedTableWidget::itemFromIndex(const QModelIndex& index) const
{
    return dynamic_cast<PropertyItem*>(QTableWidget::itemFromIndex(index));
}


bool CustomizedTableWidget::isPointingBorder(int x)
{
    int border = columnWidth(0);
    offsetX = border - x;
    return (x >= border - 2 && x <= border + 2);
}


void CustomizedTableWidget::mouseMoveEvent(QMouseEvent* event)
{
    if(isResizing){
        int border = std::max(0, event->x() + offsetX);
        setColumnWidth(0, border);
    } else if(isPointingBorder(event->x())){
        setCursor(Qt::SizeHorCursor);
    } else {
        setCursor(QCursor());
        QTableWidget::mouseMoveEvent(event);
    }
}


void CustomizedTableWidget::mousePressEvent(QMouseEvent* event)
{
    if(isPointingBorder(event->x())){
        isResizing = true;
    } else {
        QTableWidget::mousePressEvent(event);
    }
}
    

void CustomizedTableWidget::mouseReleaseEvent(QMouseEvent* event)
{
    isResizing = false;
    QTableWidget::mouseReleaseEvent(event);
}


void CustomizedTableWidget::resizeEvent(QResizeEvent* event)
{
    QTableWidget::resizeEvent(event);
    horizontalHeader()->resizeSections(QHeaderView::Stretch);
}


CustomizedItemDelegate::CustomizedItemDelegate(CustomizedTableWidget* tableWidget)
    : tableWidget(tableWidget)
{
    decimals = 2;
}


QWidget* CustomizedItemDelegate::createEditor
(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    QWidget* editor = 0;
    PropertyItem* item = tableWidget->itemFromIndex(index);
    if(item){
        ValueVariant& value = item->value;
        switch(stdx::get_variant_index(value)){

        case TYPE_INT:
            editor = QStyledItemDelegate::createEditor(parent, option, index);
            if(QSpinBox* spinBox = dynamic_cast<QSpinBox*>(editor)){
                Int& v = stdx::get<Int>(value);
                spinBox->setRange(v.min, v.max);
            }
            break;
                    
        case TYPE_DOUBLE:
            editor = QStyledItemDelegate::createEditor(parent, option, index);
            if(QDoubleSpinBox* doubleSpinBox = dynamic_cast<QDoubleSpinBox*>(editor)){
                Double& v = stdx::get<Double>(value);
                if(v.decimals >= 0){
                    doubleSpinBox->setDecimals(v.decimals);
                    doubleSpinBox->setSingleStep(pow(10.0, -v.decimals));
                }
                doubleSpinBox->setRange(v.min, v.max);
            }
            break;

        case TYPE_FILEPATH:
            editor = new FilePathEditor(const_cast<CustomizedItemDelegate*>(this), parent);
            break;

        default:
            editor = QStyledItemDelegate::createEditor(parent, option, index);
            break;
        }
    }
    return editor;
}


void CustomizedItemDelegate::updateEditorGeometry
(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    editor->setGeometry(option.rect);

    /*
      if(auto combo = dynamic_cast<QComboBox*>(editor)){
      combo->showPopup();
      }
    */
}


void CustomizedItemDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
    PropertyItem* item = tableWidget->itemFromIndex(index);
    if(item && (stdx::get_variant_index(item->value) == TYPE_FILEPATH)){
        if(FilePathEditor* fpEditor = dynamic_cast<FilePathEditor*>(editor)){
            fpEditor->setValue(stdx::get<FilePathProperty>(item->value));
            return;
        }
    }
    QStyledItemDelegate::setEditorData(editor, index);
}


void CustomizedItemDelegate::setModelData
(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    PropertyItem* item = tableWidget->itemFromIndex(index);
    if(item && (stdx::get_variant_index(item->value) == TYPE_FILEPATH)){
        if(FilePathEditor* fpEditor = dynamic_cast<FilePathEditor*>(editor)){
            item->setData(Qt::EditRole, fpEditor->value());
            return;
        }
    }
    QStyledItemDelegate::setModelData(editor, model, index);
}


QString CustomizedItemDelegate::displayText(const QVariant& value, const QLocale& locale) const
{
    if(value.type() == QVariant::Double){
        return QString::number(value.toDouble(), 'f', decimals);
    }
    return QStyledItemDelegate::displayText(value, locale);
}


void CustomizedItemDelegate::paint
(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    PropertyItem* item = tableWidget->itemFromIndex(index);
    if(item && (stdx::get_variant_index(item->value) == TYPE_DOUBLE)){
        int& d = const_cast<int&>(decimals);
        d = stdx::get<Double>(item->value).decimals;
    }
    QStyledItemDelegate::paint(painter, option, index);
}


void CustomizedItemDelegate::openFileDialog(FilePathProperty value, FilePathEditor* editor)
{
    FileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(_("Select File"));
    dialog.setViewMode(QFileDialog::List);
    if(value.isExistingFileMode()){
        dialog.setFileMode(QFileDialog::ExistingFile);
        dialog.setLabelText(QFileDialog::Accept, _("Select"));
    }else{
        dialog.setFileMode(QFileDialog::AnyFile);
        dialog.setLabelText(QFileDialog::Accept, _("OK"));
    }
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));

    dialog.updatePresetDirectories();

    filesystem::path directory;
    if(!value.baseDirectory().empty()){
        directory = fromUTF8(value.baseDirectory());
    } else {
        filesystem::path filenamePath(fromUTF8(value.filename()));
        if(filenamePath.is_absolute()){
            directory = filenamePath.parent_path();
        }
    }
    if(!directory.empty()){
        dialog.setDirectory(toUTF8(directory.string()));
    }
        
    QStringList filters;
    for(auto& filter : value.filters()){
        filters << filter.c_str();
    }
    filters << _("Any files (*)");
    dialog.setNameFilters(filters);
        
    if(dialog.exec()){
        QStringList filenames;
        filenames = dialog.selectedFiles();
        filesystem::path newDirectory(
            fromUTF8(dialog.directory().absolutePath().toStdString()));
        if(newDirectory != directory){
            value.setBaseDirectory("");
        }
        string filename(filenames.at(0).toStdString());
        if(!value.baseDirectory().empty()){
            filename = toUTF8(filesystem::path(fromUTF8(filename)).filename().string());
        }
        if(value.isExtensionRemovalModeForFileDialogSelection()){
            regex pattern1(".+\\(\\*\\.(.+)\\)");
            std::smatch match;
            for(auto& filter : value.filters()){
                if(regex_match(filter, match, pattern1)){
                    regex pattern2(format("^(.+)(\\.{})$", match.str(1)), std::regex_constants::icase);
                    if(regex_match(filename, match, pattern2)){
                        filename = regex_replace(filename, pattern2, "$1");
                        break;
                    }
                }
            }
        }
        value.setFilename(filename);
        editor->setValue(value);
        commitData(editor);
    }
}

}


ItemPropertyWidget::ItemPropertyWidget(QWidget* parent)
    : QWidget(parent)
{
    impl = new Impl(this);
}


ItemPropertyWidget::Impl::Impl(ItemPropertyWidget* self)
    : self(self)
{
    isEditingProperty = false;
    
    tableWidget = new CustomizedTableWidget(self);
    tableWidget->setFrameShape(QFrame::NoFrame);
    tableWidget->setColumnCount(2);
    tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    tableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    tableWidget->setTabKeyNavigation(true);
    tableWidget->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    tableWidget->setEditTriggers(
        /* QAbstractItemView::CurrentChanged | */
        QAbstractItemView::DoubleClicked |
        QAbstractItemView::SelectedClicked | 
        QAbstractItemView::EditKeyPressed |
        QAbstractItemView::AnyKeyPressed);

    QHeaderView* hh = tableWidget->horizontalHeader();
    QHeaderView* vh = tableWidget->verticalHeader();
    hh->hide();
    vh->hide();
    hh->setSectionResizeMode(QHeaderView::Interactive);
    vh->setSectionResizeMode(QHeaderView::ResizeToContents);
    hh->setStretchLastSection(true);

    QStyledItemDelegate* delegate = new CustomizedItemDelegate(tableWidget);
    QItemEditorFactory* factory = new QItemEditorFactory;
    
    QItemEditorCreatorBase* selectionEditorCreator =
        new QStandardItemEditorCreator<StringListComboBox>();
    factory->registerEditor(QVariant::StringList, selectionEditorCreator);
    
    delegate->setItemEditorFactory(factory);

    tableWidget->setItemDelegate(delegate);
        
    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->setContentsMargins(0, 0, 0, 0);
    vbox->addWidget(tableWidget);
    self->setLayout(vbox);

    fontPointSizeDiff = 0;
}


ItemPropertyWidget::~ItemPropertyWidget()
{
    delete impl;
}


void ItemPropertyWidget::setPropertyFunction_
(const std::type_info& type, std::function<void(Item* item, PutPropertyFunction& putProperty)> func)
{
    impl->propertyFunctions.setFunction(
        type,
        [this, func](Item* item){ func(item, *impl); });
}


bool ItemPropertyWidget::hasPropertyFunctionFor(Item* item) const
{
    return item ? impl->propertyFunctions.hasFunctionFor(item) : false;
}


void ItemPropertyWidget::setCurrentItem(Item* item)
{
    impl->setCurrentItem(item);
}


void ItemPropertyWidget::Impl::setCurrentItem(Item* item)
{
    if(item != currentItem){
        itemConnections.disconnect();
        if(item){
            itemConnections.add(
                item->sigUpdated().connect(
                    [&](){ updateProperties(); }));
            itemConnections.add(
                item->sigNameChanged().connect(
                    [&](const std::string& /* oldName */){ updateProperties(); }));
            itemConnections.add(
                item->sigDisconnectedFromRoot().connect(
                    [&](){ setCurrentItem(nullptr); }));
        }
        currentItem = item;
        updateProperties(true);
    }
}


void ItemPropertyWidget::updateProperties()
{
    impl->updateProperties(false);
}


void ItemPropertyWidget::Impl::updateProperties(bool isItemChanged)
{
    if(isEditingProperty){
        updateRequestedDuringPropertyEditing = true;

    } else {
        int currentRow;
        int currentColumn;
        if(!isItemChanged){
            currentRow = tableWidget->currentRow();
            currentColumn = tableWidget->currentColumn();
        }

        tableWidget->setRowCount(0);
        if(currentItem){
            reset();
            propertyFunctions.dispatch(currentItem);
        }

        if(!isItemChanged){
            tableWidget->setCurrentCell(currentRow, currentColumn);
        }
    }
}


void ItemPropertyWidget::resetColumnSizes()
{
    impl->tableWidget->resizeColumnsToContents();
}


void ItemPropertyWidget::setOperationMenu(MenuManager& menuManager)
{
    menuManager.addItem(_("Update"))->sigTriggered().connect(
        [&](){ updateProperties(); });
    
    menuManager.addItem(_("Reset Column Sizes"))->sigTriggered().connect(
        [&](){ resetColumnSizes(); });
}


void ItemPropertyWidget::Impl::addProperty(const std::string& name, PropertyItem* propertyItem)
{
    int row = tableWidget->rowCount();
    tableWidget->setRowCount(row + 1);

    QTableWidgetItem* nameItem = new QTableWidgetItem(name.c_str());
    nameItem->setFlags(Qt::ItemIsEnabled);
    tableWidget->setItem(row, 0, nameItem);
    
    tableWidget->setItem(row, 1, propertyItem);
}


PutPropertyFunction& ItemPropertyWidget::Impl::decimals(int d)
{
    decimals_ = d;
    return *this;
}


PutPropertyFunction& ItemPropertyWidget::Impl::min(double min)
{
    dmin = min;
    return *this;
}


PutPropertyFunction& ItemPropertyWidget::Impl::max(double max)
{
    dmax = max;
    return *this;
}


PutPropertyFunction& ItemPropertyWidget::Impl::min(int min)
{
    imin = min;
    return *this;
}


PutPropertyFunction& ItemPropertyWidget::Impl::max(int max)
{
    imax = max;
    return *this;
}


PutPropertyFunction& ItemPropertyWidget::Impl::reset()
{
    decimals_ = 2;
    dmin = -std::numeric_limits<double>::max();
    dmax = std::numeric_limits<double>::max();
    imin =std::numeric_limits<int>::min();
    imax= std::numeric_limits<int>::max();
    return *this;
}
        
void ItemPropertyWidget::Impl::operator()(const std::string& name, bool value)
{
    addProperty(name, new PropertyItem(this, value));
}


void ItemPropertyWidget::Impl::operator()
(const std::string& name, bool value, const std::function<bool(bool)>& func)
{
    addProperty(name, new PropertyItem(this, value, func));
}


void ItemPropertyWidget::Impl::operator()(const std::string& name, int value)
{
    addProperty(name, new PropertyItem(this, Int(value)));
}


void ItemPropertyWidget::Impl::operator()
(const std::string& name, int value, const std::function<bool(int)>& func)
{
    addProperty(name, new PropertyItem(this, Int(value, imin, imax), func));
}


void ItemPropertyWidget::Impl::operator()(const std::string& name, double value)
{
    addProperty(name, new PropertyItem(this, Double(value, decimals_)));
}


void ItemPropertyWidget::Impl::operator()
(const std::string& name, double value, const std::function<bool(double)>& func)
{
    addProperty(name, new PropertyItem(this, Double(value, decimals_, dmin, dmax), func));
}


void ItemPropertyWidget::Impl::operator()(const std::string& name, const std::string& value)
{
    addProperty(name, new PropertyItem(this, value));
}


void ItemPropertyWidget::Impl::operator()
(const std::string& name, const std::string& value, const std::function<bool(const std::string&)>& func)
{
    addProperty(name, new PropertyItem(this, value, func));
}


void ItemPropertyWidget::Impl::operator()(const std::string& name, const Selection& selection)
{
    addProperty(name, new PropertyItem(this, selection));
}


void ItemPropertyWidget::Impl::operator()
(const std::string& name, const Selection& selection, const std::function<bool(int which)>& func)
{
    addProperty(name, new PropertyItem(this, selection, func));
}


void ItemPropertyWidget::Impl::operator()(const std::string& name, const FilePathProperty& filepath)
{
    addProperty(name, new PropertyItem(this, filepath) );
}


void ItemPropertyWidget::Impl::operator()
(const std::string& name, const FilePathProperty& filepath, const std::function<bool(const std::string&)>& func)
{
    addProperty(name, new PropertyItem(this, filepath, func) );
}


void ItemPropertyWidget::keyPressEvent(QKeyEvent* event)
{
    if(event->modifiers() & Qt::ControlModifier){
        switch(event->key()){
        case Qt::Key_Plus:
        case Qt::Key_Semicolon:
            impl->zoomFontSize(1);
            return;
        case Qt::Key_Minus:
            impl->zoomFontSize(-1);
            return;
        default:
            break;
        }
    }
    QWidget::keyPressEvent(event);
}


void ItemPropertyWidget::Impl::zoomFontSize(int pointSizeDiff)
{
    QFont font = tableWidget->font();
    font.setPointSize(font.pointSize() + pointSizeDiff);
    tableWidget->setFont(font);
    fontPointSizeDiff += pointSizeDiff;
}
