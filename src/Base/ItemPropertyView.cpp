/**
   @author Shin'ichiro Nakaoka
*/

#include "ItemPropertyView.h"
#include "ItemTreeView.h"
#include "Item.h"
#include "ViewManager.h"
#include "MenuManager.h"
#include "PutPropertyFunction.h"
#include "SelectionListEditor.h"
#include "LazyCaller.h"
#include "AppConfig.h"
#include "MainWindow.h"
#include "Buttons.h"
#include <cnoid/ConnectionSet>
#include <cnoid/ExecutablePath>
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
#include <QFileDialog>
#include <cmath>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

bool TRACE_FUNCTIONS = false;

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

template<class ValueType> class ReturnTrue {
public:
    typedef bool result_type;
    std::function<void(ValueType)> func;
    ReturnTrue(std::function<void(ValueType)> func) : func(func) { }
    bool operator()(ValueType value) const { func(value); return true; }
};

enum TypeId { TYPE_BOOL, TYPE_INT, TYPE_DOUBLE, TYPE_STRING, TYPE_SELECTION, TYPE_FILEPATH };

struct Property {
    Property(const string& name, ValueVariant value)
        : name(name), value(value), hasValidFunction(false) { }
    Property(const string& name, ValueVariant value, FunctionVariant func)
        : name(name), value(value), func(func), hasValidFunction(true) { }
    string name;
    ValueVariant value;
    FunctionVariant func;
    bool hasValidFunction;
};
typedef std::shared_ptr<Property> PropertyPtr;


class PropertyItem : public QTableWidgetItem
{
public:
    ItemPropertyViewImpl* itemPropertyViewImpl;
    ValueVariant value;
    FunctionVariant func;
    bool hasValidFunction;

    PropertyItem(ItemPropertyViewImpl* viewImpl, ValueVariant value);
    PropertyItem(ItemPropertyViewImpl* viewImpl, ValueVariant value, FunctionVariant func);
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
    CustomizedTableWidget(QWidget* parent) : QTableWidget(parent) {
        setMouseTracking(true);
        isResizing = false;
    }
        
    PropertyItem* itemFromIndex(const QModelIndex& index) const {
        return dynamic_cast<PropertyItem*>(QTableWidget::itemFromIndex(index));
    }

    bool isPointingBorder(int x){
        int border = columnWidth(0);
        offsetX = border - x;
        return (x >= border - 2 && x <= border + 2);
    }

    virtual void mouseMoveEvent(QMouseEvent* event) {
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

    virtual void mousePressEvent(QMouseEvent* event){
        if(isPointingBorder(event->x())){
            isResizing = true;
        } else {
            QTableWidget::mousePressEvent(event);
        }
    }
    
    virtual void mouseReleaseEvent(QMouseEvent* event){
        isResizing = false;
        QTableWidget::mouseReleaseEvent(event);
    }

    virtual void resizeEvent(QResizeEvent* event){
        QTableWidget::resizeEvent(event);
        horizontalHeader()->resizeSections(QHeaderView::Stretch);
    }
};

    
class CustomizedItemDelegate : public QStyledItemDelegate
{
public:
    CustomizedTableWidget* tableWidget;
    int decimals;
        
    CustomizedItemDelegate(CustomizedTableWidget* tableWidget)
        : tableWidget(tableWidget)
    {
        decimals = 2;
    }

    virtual QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override
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

    virtual void setEditorData(QWidget* editor, const QModelIndex& index) const override
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

    virtual void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override
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

    virtual QString displayText(const QVariant& value, const QLocale& locale) const override
    {
        if(value.type() == QVariant::Double){
            return QString::number(value.toDouble(), 'f', decimals);
        }
        return QStyledItemDelegate::displayText(value, locale);
    }

    void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override
    {
        PropertyItem* item = tableWidget->itemFromIndex(index);
        if(item && (stdx::get_variant_index(item->value) == TYPE_DOUBLE)){
            int& d = const_cast<int&>(decimals);
            d = stdx::get<Double>(item->value).decimals;
        }
        QStyledItemDelegate::paint(painter, option, index);
    }

    void openFileDialog(FilePathProperty value, FilePathEditor* editor)
    {
        QFileDialog dialog(MainWindow::instance());
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

        filesystem::path directory;
        if(value.baseDirectory().empty()){
            filesystem::path filenamePath(value.filename());
            if(filenamePath.is_absolute()){
                directory = filenamePath.parent_path();
            } else {
                directory = AppConfig::archive()->get("currentFileDialogDirectory", shareDirectory());
            }
        } else {
            directory = value.baseDirectory();
        }
        dialog.setDirectory(directory.string().c_str());
        
        QStringList filters;
        for(auto& filter : value.filters()){
            filters << filter.c_str();
        }
        filters << _("Any files (*)");
        dialog.setNameFilters(filters);
        
        if(dialog.exec()){
            QStringList filenames;
            filenames = dialog.selectedFiles();
            filesystem::path newDirectory(dialog.directory().absolutePath().toStdString());
            if(newDirectory != directory){
                AppConfig::archive()->writePath("currentFileDialogDirectory", newDirectory.string());
                value.setBaseDirectory("");
            }
            string filename(filenames.at(0).toStdString());
            if(value.baseDirectory().empty()){
                value.setFilename(filename);
            } else {
                value.setFilename(filesystem::path(filename).filename().string());
            }
            editor->setValue(value);

            commitData(editor);
        }
    }
};


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
        return (filesystem::path(currentValue.baseDirectory()) / lineEdit->text().toStdString()).string().c_str();
    }
}

}


namespace cnoid {

class ItemPropertyViewImpl : public PutPropertyFunction
{
public:
    ItemPropertyViewImpl(ItemPropertyView* self);
    ~ItemPropertyViewImpl();
        
    ItemPropertyView* self;

    int decimals_;
    double dmin;
    double dmax;
    int imin;
    int imax;

    CustomizedTableWidget* tableWidget;
    int fontPointSizeDiff;
        
    ItemPtr currentItem;
    ConnectionSet itemConnections;
    int tmpListIndex;

    bool isEditingProperty;
    bool updateRequestedDuringPropertyEditing;

    std::vector<PropertyPtr> properties;
        
    bool isPressedPathValid;

    Connection selectionChangedConnection;

    // PutPropertyFunction's virtual functions
    PutPropertyFunction& decimals(int d) {
        decimals_ = d;
        return *this;
    }
    PutPropertyFunction& min(double min) {
        dmin = min;
        return *this;
    }
    PutPropertyFunction& max(double max) {
        dmax = max;
        return *this;
    }
    PutPropertyFunction& min(int min){
        imin = min;
        return *this;
    }
    PutPropertyFunction& max(int max){
        imax = max;
        return *this;
    }
    PutPropertyFunction& reset(){
        decimals_ = 2;
        dmin = -std::numeric_limits<double>::max();
        dmax = std::numeric_limits<double>::max();
        imin =std::numeric_limits<int>::min();
        imax= std::numeric_limits<int>::max();
        return *this;
    }
        
    virtual void operator()(const std::string& name, bool value){
        addProperty(name, new PropertyItem(this, value));
    }
    virtual void operator()(const std::string& name, bool value, const std::function<bool(bool)>& func) {
        addProperty(name, new PropertyItem(this, value, func));
    }
    virtual void operator()(const std::string& name, bool value,
                            const std::function<void(bool)>& func, bool /* forceUpdate */) {
        addProperty(name, new PropertyItem
                    (this, value, std::function<bool(bool)>(ReturnTrue<bool>(func))));
    }
    virtual void operator()(const std::string& name, int value){
        addProperty(name, new PropertyItem(this, Int(value)));
    }
    virtual void operator()(const std::string& name, int value, const std::function<bool(int)>& func){
        addProperty(name, new PropertyItem(this, Int(value, imin, imax), func));
    }
    virtual void operator()(const std::string& name, int value,
                            const std::function<void(int)>& func, bool /* forceUpdate */){
        addProperty(name, new PropertyItem(this, Int(value, imin, imax),
                                           std::function<bool(int)>(ReturnTrue<int>(func))));
    }
    virtual void operator()(const std::string& name, double value){
        addProperty(name, new PropertyItem(this, Double(value, decimals_)));
    }
    virtual void operator()(const std::string& name, double value,
                            const std::function<bool(double)>& func){
        addProperty(name, new PropertyItem(this, Double(value, decimals_, dmin, dmax), func));
    }
    virtual void operator()(const std::string& name, double value,
                            const std::function<void(double)>& func, bool /* forceUpdate */){
        addProperty(name, new PropertyItem(this, Double(value, decimals_, dmin, dmax),
                                           std::function<bool(double)>(ReturnTrue<double>(func))));
    }
    virtual void operator()(const std::string& name, const std::string& value){
        addProperty(name, new PropertyItem(this, value));
    }
    virtual void operator()(const std::string& name, const std::string& value,
                            const std::function<bool(const std::string&)>& func){
        addProperty(name, new PropertyItem(this, value, func));
    }
    virtual void operator()(const std::string& name, const std::string& value,
                            const std::function<void(const std::string&)>& func, bool /* forceUpdate */){
        addProperty(name, new PropertyItem
                    (this, value, std::function<bool(const std::string&)>(ReturnTrue<const std::string&>(func))));
    }
    void operator()(const std::string& name, const Selection& selection){
        addProperty(name, new PropertyItem(this, selection));
    }
    void operator()(const std::string& name, const Selection& selection,
                    const std::function<bool(int which)>& func){
        addProperty(name, new PropertyItem(this, selection, func));
    }
    void operator()(const std::string& name, const Selection& selection,
                    const std::function<void(int which)>& func, bool /* forceUpdate */){
        addProperty(name, new PropertyItem
                    (this, selection,  std::function<bool(int)>(ReturnTrue<int>(func))));
    }
    void operator()(const std::string& name, const FilePathProperty& filepath){
        addProperty(name, new PropertyItem(this, filepath) );
    }
    void operator()(const std::string& name, const FilePathProperty& filepath,
                    const std::function<bool(const std::string&)>& func){
        addProperty(name, new PropertyItem(this, filepath, func) );
    }
    void operator()(const std::string& name, const FilePathProperty& filepath,
                    const std::function<void(const std::string&)>& func, bool /* forceUpdate */){
        addProperty(name, new PropertyItem(this, filepath, std::function<bool(const std::string&)>(ReturnTrue<const std::string&>(func))));
    }

    void clear();
    void updateProperties(bool isItemChanged = false);
    void addProperty(const std::string& name, PropertyItem* propertyItem);
    void onItemSelectionChanged(const ItemList<>& items);
    void zoomFontSize(int pointSizeDiff);
};

}


PropertyItem::PropertyItem(ItemPropertyViewImpl* viewImpl, ValueVariant value)
    : itemPropertyViewImpl(viewImpl),
      value(value)
{
    setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    hasValidFunction = false;
}


PropertyItem::PropertyItem(ItemPropertyViewImpl* viewImpl, ValueVariant value, FunctionVariant func)
    : itemPropertyViewImpl(viewImpl),
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
                    filename = filesystem::path(filename).filename().string();
                }
                return filename.c_str();
            }
        }
    } else if(role == Qt::ToolTipRole){
        if(stdx::get_variant_index(value) == TYPE_FILEPATH){
            const FilePathProperty& f = stdx::get<FilePathProperty>(value);
            if(!f.isFullpathDisplayMode()){
                string fullpath = f.filename();
                string filename = filesystem::path(fullpath).filename().string();
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
    itemPropertyViewImpl->isEditingProperty = true;
    itemPropertyViewImpl->updateRequestedDuringPropertyEditing = false;
    
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
            if(!itemPropertyViewImpl->updateRequestedDuringPropertyEditing){
                itemPropertyViewImpl->currentItem->notifyUpdate();
            }
        }
        if(itemPropertyViewImpl->updateRequestedDuringPropertyEditing){
            auto itemPropertyViewImpl = this->itemPropertyViewImpl;
            callLater([itemPropertyViewImpl](){ itemPropertyViewImpl->updateProperties(); });
        }
    }

    itemPropertyViewImpl->isEditingProperty = false;
}


void ItemPropertyView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<ItemPropertyView>(
        "ItemPropertyView", N_("Property"), ViewManager::SINGLE_DEFAULT);
}


ItemPropertyView::ItemPropertyView()
{
    impl = new ItemPropertyViewImpl(this);
}


ItemPropertyViewImpl::ItemPropertyViewImpl(ItemPropertyView* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::LEFT_BOTTOM);

    isPressedPathValid = false;
    isEditingProperty = false;
    
    tableWidget = new CustomizedTableWidget(self);
    tableWidget->setFrameShape(QFrame::NoFrame);
    tableWidget->setColumnCount(2);
    tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    tableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    tableWidget->setTabKeyNavigation(true);
    tableWidget->setEditTriggers(
        QAbstractItemView::DoubleClicked |
        QAbstractItemView::SelectedClicked | 
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
    
    QItemEditorCreatorBase* selectionListCreator =
        new QStandardItemEditorCreator<SelectionListEditor>();
    factory->registerEditor(QVariant::StringList, selectionListCreator);
    
    delegate->setItemEditorFactory(factory);

    tableWidget->setItemDelegate(delegate);
        
    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->addWidget(tableWidget);
    self->setLayout(vbox);

    selectionChangedConnection =
        ItemTreeView::mainInstance()->sigSelectionChanged().connect(
            [&](const ItemList<>& items){ onItemSelectionChanged(items); });

    fontPointSizeDiff = 0;
    MappingPtr config = AppConfig::archive()->openMapping("ItemPropertyView");
    int storedFontPointSizeDiff;
    if(config->read("fontZoom", storedFontPointSizeDiff)){
        zoomFontSize(storedFontPointSizeDiff);
    }

}


ItemPropertyView::~ItemPropertyView()
{
    if(TRACE_FUNCTIONS){
        cout << "ItemPropertyView::~ItemPropertyView()" << endl;
    }
    
    delete impl;
}


ItemPropertyViewImpl::~ItemPropertyViewImpl()
{
    if(TRACE_FUNCTIONS){
        cout << "ItemPropertyViewImpl::~ItemPropertyView()Impl" << endl;
    }

    itemConnections.disconnect();
    selectionChangedConnection.disconnect();
}


void ItemPropertyViewImpl::clear()
{
    if(TRACE_FUNCTIONS){
        cout << "ItemPropertyView::clear()" << endl;
    }
    
    itemConnections.disconnect();
    currentItem = 0;
    updateProperties(true);
}


void ItemPropertyViewImpl::updateProperties(bool isItemChanged)
{
    if(TRACE_FUNCTIONS){
        cout << "ItemPropertyView::updateProperties()" << endl;
    }

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
        tmpListIndex = 0;
        properties.clear();
        if(currentItem){
            reset();
            currentItem->putProperties(*this);
        }

        if(!isItemChanged){
            tableWidget->setCurrentCell(currentRow, currentColumn);
        }
    }
}


void ItemPropertyViewImpl::addProperty(const std::string& name, PropertyItem* propertyItem)
{
    int row = tableWidget->rowCount();
    tableWidget->setRowCount(row + 1);

    QTableWidgetItem* nameItem = new QTableWidgetItem(name.c_str());
    nameItem->setFlags(Qt::ItemIsEnabled);
    tableWidget->setItem(row, 0, nameItem);
    
    tableWidget->setItem(row, 1, propertyItem);
}


void ItemPropertyViewImpl::onItemSelectionChanged(const ItemList<>& items)
{
    if(TRACE_FUNCTIONS){
        cout << "ItemPropertyView::onItemSelectionChanged()" << endl;
    }

    Item* item = items.toSingle();

    if(item != currentItem){
        itemConnections.disconnect();
        currentItem = item;
        if(item){
            itemConnections.add(
                item->sigUpdated().connect(
                    [&](){ updateProperties(); }));
            itemConnections.add(
                item->sigNameChanged().connect(
                    [&](const std::string& /* oldName */){ updateProperties(); }));
            itemConnections.add(
                item->sigDetachedFromRoot().connect(
                    [&](){ clear(); }));
        }
        updateProperties(true);
    }
}


void ItemPropertyView::keyPressEvent(QKeyEvent* event)
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
    View::keyPressEvent(event);
}


void ItemPropertyViewImpl::zoomFontSize(int pointSizeDiff)
{
    QFont font = tableWidget->font();
    font.setPointSize(font.pointSize() + pointSizeDiff);
    tableWidget->setFont(font);
    fontPointSizeDiff += pointSizeDiff;
    AppConfig::archive()->openMapping("ItemPropertyView")->write("fontZoom", fontPointSizeDiff);
}


void ItemPropertyView::onAttachedMenuRequest(MenuManager& menuManager)
{
    menuManager.addItem(_("Update"))->sigTriggered().connect(
        [&](){ impl->updateProperties(); });
    
    menuManager.addItem(_("Reset Column Sizes"))->sigTriggered().connect(
        [&](){ impl->tableWidget->resizeColumnsToContents(); });
}
