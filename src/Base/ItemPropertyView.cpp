/**
   @author Shin'ichiro Nakaoka
*/

#include "ItemPropertyView.h"
#include "ItemTreeView.h"
#include "Item.h"
#include "ItemList.h"
#include "ItemManager.h"
#include "ViewManager.h"
#include "MenuManager.h"
#include "MessageView.h"
#include "PutPropertyFunction.h"
#include "SelectionListEditor.h"
#include "LazyCaller.h"
#include "AppConfig.h"
#include "Archive.h"
#include "MainWindow.h"
#include <cnoid/ConnectionSet>
#include <cnoid/ExecutablePath>
#include <QTableWidget>
#include <QHeaderView>
#include <QBoxLayout>
#include <QDoubleSpinBox>
#include <QStyledItemDelegate>
#include <QItemEditorFactory>
#include <QStandardItemEditorCreator>
#include <QKeyEvent>
#include <QPainter>
#include <QApplication>
#include <QFileDialog>
#include <boost/variant.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <cmath>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;


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

typedef boost::variant<bool, Int, Double, string, Selection, FilePath> ValueVariant;

typedef boost::variant<std::function<bool(bool)>,
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
    PropertyItem(ItemPropertyViewImpl* viewImpl, ValueVariant value);
    PropertyItem(ItemPropertyViewImpl* viewImpl, ValueVariant value, FunctionVariant func);

    virtual QVariant data(int role) const;
    virtual void setData(int role, const QVariant& qvalue);

    ItemPropertyViewImpl* itemPropertyViewImpl;
    ValueVariant value;
    FunctionVariant func;
    bool hasValidFunction;
    bool buttonState;   // When the value type is FilePath
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
};

    
class CustomizedItemDelegate : public QStyledItemDelegate
{
public:
    CustomizedTableWidget* tableWidget;
    int decimals;
        
    CustomizedItemDelegate(CustomizedTableWidget* tableWidget)
        : tableWidget(tableWidget) {
        decimals = 2;}

    virtual QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const {

        QWidget* editor = QStyledItemDelegate::createEditor(parent, option, index);
        PropertyItem* item = tableWidget->itemFromIndex(index);
        if(item){
            if(QSpinBox* spinBox = dynamic_cast<QSpinBox*>(editor)){
                ValueVariant& value = item->value;
                if(value.which() == TYPE_INT){
                    Int& v = boost::get<Int>(value);
                    spinBox->setRange(v.min, v.max);
                }
            } else if(QDoubleSpinBox* doubleSpinBox = dynamic_cast<QDoubleSpinBox*>(editor)){
                ValueVariant& value = item->value;
                if(value.which() == TYPE_DOUBLE){
                    Double& v = boost::get<Double>(value);
                    if(v.decimals >= 0){
                        doubleSpinBox->setDecimals(v.decimals);
                        doubleSpinBox->setSingleStep(pow(10.0, -v.decimals));
                    }
                    doubleSpinBox->setRange(v.min, v.max);
                }
            }
        }
        return editor;
    }

    virtual QString displayText(const QVariant& value, const QLocale& locale) const {
        if(value.type() == QVariant::Double){
            return QString::number(value.toDouble(), 'f', decimals);
        }
        return QStyledItemDelegate::displayText(value, locale);
    }

    void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const {
            
        PropertyItem* item = tableWidget->itemFromIndex(index);
        if(item && item->value.which() == TYPE_FILEPATH){
            FilePath& v = boost::get<FilePath>(item->value);
            QString strText = QString(v.filename.c_str());
            QStyleOptionButton fileButton;
            fileButton.icon = QApplication::style()->standardIcon( QStyle::SP_FileDialogStart );
            QRect r = option.rect;
            fileButton.iconSize = QSize(r.height(), r.height());
            if(item->buttonState)
                fileButton.state = QStyle::State_Sunken | QStyle::State_Enabled;
            else
                fileButton.state = QStyle::State_Raised | QStyle::State_Enabled;
            fileButton.features = QStyleOptionButton::None;
            fileButton.rect = QRect(r.left() + r.width() - r.height(), r.top(), r.height(), r.height());
            QApplication::style()->drawControl( QStyle::CE_PushButton, &fileButton, painter);
            QRect rect(r.left(), r.top(), r.width()-r.height(), r.height());
            painter->drawText(rect, Qt::TextWrapAnywhere, strText );
            return;
        }
        if(item && item->value.which() == TYPE_DOUBLE){
            int& d = const_cast<int&>(decimals);
            d = boost::get<Double>(item->value).decimals;
        }
        QStyledItemDelegate::paint(painter, option, index);
    }

    bool editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index) {
        PropertyItem* item = tableWidget->itemFromIndex(index);
        if(item && item->value.which() == TYPE_FILEPATH){
            QMouseEvent * e = (QMouseEvent *)event;
            QRect r = option.rect;
            QRect rect(r.left() + r.width() - r.height(), r.top(), r.height(), r.height());
            if( rect.contains( e->x(), e->y() )){
                if( e->type() == QEvent::MouseButtonRelease ){
                    item->buttonState = false;
                    tableWidget->repaint(rect);
                    FilePath& v = boost::get<FilePath>(item->value);
                    if( openFileDialog(v) ){
                        item->setData( Qt::EditRole, v.filename.c_str() );
                    }
                }else if( e->type() == QEvent::MouseButtonPress){
                    item->buttonState = true;
                    tableWidget->repaint(rect);
                }
                return true;
            }
        }
        return QStyledItemDelegate::editorEvent(event, model, option, index);
    }

    bool openFileDialog(FilePath& v){
        QFileDialog dialog(MainWindow::instance());
        dialog.setWindowTitle(_("Select File"));
        dialog.setViewMode(QFileDialog::List);
        dialog.setFileMode(QFileDialog::ExistingFile);
        dialog.setLabelText(QFileDialog::Accept, _("Select"));
        dialog.setLabelText(QFileDialog::Reject, _("Cancel"));
        if(!v.directory.empty())
            dialog.setDirectory(v.directory.c_str());
        else
            dialog.setDirectory(AppConfig::archive()->get("currentFileDialogDirectory", shareDirectory()).c_str());
        QStringList filters;
        for(int i=0; i<v.filters.size(); i++)
            filters << v.filters[i].c_str();
        filters << _("Any files (*)");
        dialog.setNameFilters(filters);
        if(dialog.exec()){
            QStringList fileNames;
            fileNames = dialog.selectedFiles();
            v.filename = fileNames.at(0).toStdString();
            return true;
        }
        return false;
    }

};
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
                            const std::function<void(bool)>& func, bool forceUpdate) {
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
                            const std::function<void(int)>& func, bool forceUpdate){
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
                            const std::function<void(double)>& func, bool forceUpdate){
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
                            const std::function<void(const std::string&)>& func, bool forceUpdate){
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
                    const std::function<void(int which)>& func, bool forceUpdate){
        addProperty(name, new PropertyItem
                    (this, selection,  std::function<bool(int)>(ReturnTrue<int>(func))));
    }
    void operator()(const std::string& name, const FilePath& filePath){
        addProperty(name, new PropertyItem(this, filePath) );
    }
    void operator()(const std::string& name, const FilePath& filePath,
                    const std::function<bool(const std::string&)>& func){
        addProperty(name, new PropertyItem(this, filePath, func) );
    }
    void operator()(const std::string& name, const FilePath& filePath,
                    const std::function<void(const std::string&)>& func, bool forceUpdate){
        addProperty(name, new PropertyItem(this, filePath, std::function<bool(const std::string&)>(ReturnTrue<const std::string&>(func))));
    }

    void clear();
    void updateProperties();
    void addProperty(const std::string& name, PropertyItem* propertyItem);
    void onItemSelectionChanged(const ItemList<>& items);
    void zoomFontSize(int pointSizeDiff);
};
}



PropertyItem::PropertyItem(ItemPropertyViewImpl* viewImpl, ValueVariant value)
    : itemPropertyViewImpl(viewImpl),
      value(value),
      buttonState(false)
{
    setFlags(Qt::ItemIsEnabled);
    hasValidFunction = false;
}


PropertyItem::PropertyItem(ItemPropertyViewImpl* viewImpl, ValueVariant value, FunctionVariant func)
    : itemPropertyViewImpl(viewImpl),
      value(value),
      func(func),
      buttonState(false)
{
    setFlags(Qt::ItemIsEnabled|Qt::ItemIsEditable);
    hasValidFunction = true;
}


QVariant PropertyItem::data(int role) const
{
    if(role == Qt::DisplayRole || role == Qt::EditRole){
        switch(value.which()){
        case TYPE_BOOL:      return boost::get<bool>(value);
        case TYPE_INT:       return boost::get<Int>(value).value;
        case TYPE_DOUBLE:    return boost::get<Double>(value).value;
        case TYPE_STRING:    return boost::get<string>(value).c_str();

        case TYPE_SELECTION:
        {
            const Selection& s = boost::get<Selection>(value);
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
            return boost::get<FilePath>(value).filename.c_str();
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

        try {
            switch(qvalue.type()){
                
            case QVariant::Bool:
                accepted = boost::get< std::function<bool(bool)> >(func)(qvalue.toBool());
                break;
                
            case QVariant::String:
                accepted = boost::get< std::function<bool(const string&)> >(func)(qvalue.toString().toStdString());
                break;
                
            case QVariant::Int:
                accepted = boost::get< std::function<bool(int)> >(func)(qvalue.toInt());
                break;
                
            case QVariant::Double:
                accepted = boost::get< std::function<bool(double)> >(func)(qvalue.toDouble());
                break;
                
            case QVariant::StringList:
            {
                const QStringList& slist = qvalue.toStringList();
                if(!slist.empty()){
                    accepted = boost::get< std::function<bool(int)> >(func)(slist[0].toInt());
                }
            }
            break;
            
            default:
                break;
            }
            
        } catch(const boost::bad_lexical_cast& ex) {
            
        }
        
        if(accepted){
            if(!itemPropertyViewImpl->updateRequestedDuringPropertyEditing){
                itemPropertyViewImpl->currentItem->notifyUpdate();
            }
        }
        if(itemPropertyViewImpl->updateRequestedDuringPropertyEditing){
            callLater(std::bind(&ItemPropertyViewImpl::updateProperties, itemPropertyViewImpl));
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
    tableWidget->setSelectionMode(QAbstractItemView::NoSelection);

    QHeaderView* hh = tableWidget->horizontalHeader();
    QHeaderView* vh = tableWidget->verticalHeader();
    hh->hide();
    vh->hide();
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
    hh->setResizeMode(QHeaderView::Fixed);
    vh->setResizeMode(QHeaderView::ResizeToContents);    
#else
    hh->setSectionResizeMode(QHeaderView::Stretch);
    vh->setSectionResizeMode(QHeaderView::ResizeToContents);
#endif
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
            std::bind(&ItemPropertyViewImpl::onItemSelectionChanged, this, _1));

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
    updateProperties();
}


void ItemPropertyViewImpl::updateProperties()
{
    if(TRACE_FUNCTIONS){
        cout << "ItemPropertyView::updateProperties()" << endl;
    }

    if(isEditingProperty){
        updateRequestedDuringPropertyEditing = true;

    } else {
        tableWidget->setRowCount(0);
        
        tmpListIndex = 0;
        properties.clear();
        if(currentItem){
            reset();
            currentItem->putProperties(*this);
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
                    std::bind(&ItemPropertyViewImpl::updateProperties, this)));
            itemConnections.add(
                item->sigNameChanged().connect(
                    std::bind(&ItemPropertyViewImpl::updateProperties, this)));
            itemConnections.add(
                item->sigDetachedFromRoot().connect(
                    std::bind(&ItemPropertyViewImpl::clear, this)));
        }
        updateProperties();
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
        defaut:
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
        std::bind(&ItemPropertyViewImpl::updateProperties, impl));
    menuManager.addItem(_("Reset Column Sizes"))->sigTriggered().connect(
        std::bind(&QTableWidget::resizeColumnsToContents, impl->tableWidget));
}
