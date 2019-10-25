#include "ManipulatorVariableListViewBase.h"
#include "ManipulatorVariableListItemBase.h"
#include <cnoid/ManipulatorVariableList>
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/TargetItemPicker>
#include <cnoid/Buttons>
#include <QBoxLayout>
#include <QLabel>
#include <QTableView>
#include <QHeaderView>
#include <QAbstractTableModel>
#include <QStyledItemDelegate>
#include <QComboBox>
#include <QKeyEvent>
#include <QMouseEvent>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

constexpr int NumColumns = 4;
constexpr int IdColumn = 0;
constexpr int ValueTypeColumn = 1;
constexpr int ValueColumn = 2;
constexpr int NoteColumn = 3;

class VariableListModel : public QAbstractTableModel
{
public:
    ManipulatorVariableListPtr variables;
    
    VariableListModel(QObject* parent);
    void setVariableList(ManipulatorVariableList* variables);
    bool isValid() const;
    int numVariables() const;
    ManipulatorVariable* variableAtRow(int row);
    int rowOfVariable(ManipulatorVariable* variable) const;
    virtual int rowCount(const QModelIndex& parent) const override;
    virtual int columnCount(const QModelIndex& parent) const override;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    virtual QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    virtual Qt::ItemFlags flags(const QModelIndex& index) const override;
    virtual QVariant data(const QModelIndex& index, int role) const override;
    virtual bool setData(const QModelIndex& index, const QVariant& value, int role) override;
    void addVariable(int row, ManipulatorVariable* variable, bool doInsert);
    void removeVariables(QModelIndexList selected);
    void notifyVariableUpdate(ManipulatorVariable* variable);
};

class CustomizedItemDelegate : public QStyledItemDelegate
{
public:
    ManipulatorVariableListViewBase::Impl* view;
    
    CustomizedItemDelegate(ManipulatorVariableListViewBase::Impl* view);
    virtual QWidget* createEditor(
        QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual void updateEditorGeometry(
        QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual void setEditorData(QWidget* editor, const QModelIndex& index) const override;
    virtual void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;
};

}

namespace cnoid {

class ManipulatorVariableListViewBase::Impl : public QTableView
{
public:
    ManipulatorVariableListViewBase* self;
    TargetItemPicker<ManipulatorVariableListItemBase> targetItemPicker;
    ManipulatorVariableListItemBasePtr targetItem;
    ManipulatorVariableListPtr variables;
    VariableListModel* variableListModel;
    QLabel targetLabel;
    PushButton addButton;
    MenuManager contextMenuManager;

    Impl(ManipulatorVariableListViewBase* self);
    void setManipulatorVariableListItem(ManipulatorVariableListItemBase* item);
    void addVariableIntoCurrentIndex(bool doInsert);
    void addVariable(int row, bool doInsert);
    void removeSelectedVariables();
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    void showContextMenu(int row, QPoint globalPos);
};

}


VariableListModel::VariableListModel(QObject* parent)
    : QAbstractTableModel(parent)
{

}


void VariableListModel::setVariableList(ManipulatorVariableList* variables)
{
    beginResetModel();
    this->variables = variables;
    endResetModel();
}


bool VariableListModel::isValid() const
{
    return variables != nullptr;
}


int VariableListModel::numVariables() const
{
    return variables ?  variables->numVariables() : 0;
}


ManipulatorVariable* VariableListModel::variableAtRow(int row)
{
    return variables->variableAt(row);
}


int VariableListModel::rowOfVariable(ManipulatorVariable* variable) const
{
    if(variables){
        return variables->indexOf(variable);
    }
    return -1;
}


int VariableListModel::rowCount(const QModelIndex& parent) const
{
    int n = 0;
    if(!parent.isValid()){
        n = numVariables();
    }
    if(n == 0){ // to show an empty row
        n = 1;
    }
    return n;
}


int VariableListModel::columnCount(const QModelIndex& parent) const
{
    return NumColumns;
}
        

QVariant VariableListModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(role == Qt::DisplayRole){
        if(orientation == Qt::Horizontal){
            switch(section){
            case IdColumn:
                return _("ID");
            case ValueTypeColumn:
                return _("Type");
            case ValueColumn:
                return _("Value");
            case NoteColumn:
                return _("Note");
            }
        } else {
            return QString::number(section);
        }
    }
    else if(role == Qt::TextAlignmentRole){
        if(orientation == Qt::Horizontal){
            return Qt::AlignLeft + Qt::AlignVCenter;
        }
    }
    return QVariant();
}


QModelIndex VariableListModel::index(int row, int column, const QModelIndex& parent) const
{
    if(!variables || parent.isValid()){
        return QModelIndex();
    }
    if(row < numVariables()){
        auto variable = const_cast<VariableListModel*>(this)->variableAtRow(row);
        return createIndex(row, column, variable);
    }
    return QModelIndex();
}
    

Qt::ItemFlags VariableListModel::flags(const QModelIndex& index) const
{
    return QAbstractTableModel::flags(index) | Qt::ItemIsEditable;
}


QVariant VariableListModel::data(const QModelIndex& index, int role) const
{
    auto variable = static_cast<ManipulatorVariable*>(index.internalPointer());

    if(!variable || !index.isValid()){
        return QVariant();
    }

    int column = index.column();
    if(role == Qt::DisplayRole || role == Qt::EditRole){
        if(column == IdColumn){
            return variable->id().label().c_str();

        } else if(column == ValueTypeColumn){
            switch(variable->valueTypeId()){
            case ManipulatorVariable::Bool:
                return "Bool";
            case ManipulatorVariable::Int:
                return "Integer";
            case ManipulatorVariable::Double:
                return "Real";
            case ManipulatorVariable::String:
                return "String";
            default:
                return "Unknown";
            }

        } else if(column == ValueColumn){
            return variable->valueString().c_str();

        } else if(column == NoteColumn){
            return variable->note().c_str();
        }
    } else if(role == Qt::TextAlignmentRole){
        return (Qt::AlignLeft + Qt::AlignVCenter);
    }
            
    return QVariant();
}


bool VariableListModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
    if(index.isValid() && role == Qt::EditRole){
        auto variable = static_cast<ManipulatorVariable*>(index.internalPointer());
        int column = index.column();
        if(column == IdColumn){
            bool isInt;
            auto stringId = value.toString();
            int intId = stringId.toInt(&isInt);
            if(isInt){
                variables->resetId(variable, intId);
            } else {
                variables->resetId(variable, stringId.toStdString());
            }
            Q_EMIT dataChanged(index, index, {role});

        } else if(column == ValueTypeColumn){
            variable->changeValueType(value.toInt());
            Q_EMIT dataChanged(index, index, {role});

        } else if(column == ValueColumn){
            switch(variable->valueTypeId()){
            case ManipulatorVariable::Bool:
                variable->setValue(value.toBool());
                break;
            case ManipulatorVariable::Int:
                variable->setValue(value.toInt());
                break;
            case ManipulatorVariable::Double:
                variable->setValue(value.toDouble());
                break;
            case ManipulatorVariable::String:
                variable->setValue(value.toString().toStdString());
                break;
            default:
                break;
            }
            Q_EMIT dataChanged(index, index, {role});

        } else if(column == NoteColumn){
            variable->setNote(value.toString().toStdString());
            Q_EMIT dataChanged(index, index, {role});
        }
    }
    return false;
}


void VariableListModel::addVariable(int row, ManipulatorVariable* variable, bool doInsert)
{
    if(variables){
        int newVariableRow = doInsert ? row : row + 1;
        if(numVariables() == 0){
            // Remove the empty row first
            beginRemoveRows(QModelIndex(), 0, 0);
            endRemoveRows();
        }
        beginInsertRows(QModelIndex(), newVariableRow, newVariableRow);
        variables->insert(newVariableRow, variable);
        endInsertRows();
    }
}


void VariableListModel::removeVariables(QModelIndexList selected)
{
    if(variables){
        std::sort(selected.begin(), selected.end());
        int numRemoved = 0;
        for(auto& index : selected){
            int row = index.row() - numRemoved;
            beginRemoveRows(QModelIndex(), row, row);
            variables->removeAt(row);
            ++numRemoved;
            endRemoveRows();
        }
        if(variables->numVariables() == 0){
            // This is necessary to show the empty row
            beginResetModel();
            endResetModel();
        }
    }
}


void VariableListModel::notifyVariableUpdate(ManipulatorVariable* variable)
{
    int row = rowOfVariable(variable);
    if(row >= 0){
        auto index1 = index(row, IdColumn, QModelIndex());
        auto index2 = index(row, NoteColumn, QModelIndex());
        Q_EMIT dataChanged(index1, index2, { Qt::EditRole });
    }
}
        

CustomizedItemDelegate::CustomizedItemDelegate(ManipulatorVariableListViewBase::Impl* view)
    : QStyledItemDelegate(view),
      view(view)
{

}


QWidget* CustomizedItemDelegate::createEditor
(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    QWidget* editor = nullptr;
    
    auto variable = static_cast<ManipulatorVariable*>(index.internalPointer());
    
    switch(index.column()){
    case ValueTypeColumn:
    {
        auto combo = new QComboBox(parent);
        combo->addItem("Bool");
        combo->addItem("Integer");
        combo->addItem("Real");
        combo->addItem("String");
        combo->setCurrentIndex(variable->valueTypeId());
        editor = combo;
        break;
    }
    default:
        editor = QStyledItemDelegate::createEditor(parent, option, index);
        break;
    }

    return editor;
}


void CustomizedItemDelegate::updateEditorGeometry
(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    editor->setGeometry(option.rect);
}


void CustomizedItemDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
    QStyledItemDelegate::setEditorData(editor, index);
}


void CustomizedItemDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    switch(index.column()){
    case ValueTypeColumn:
        if(auto combo = dynamic_cast<QComboBox*>(editor)){
            model->setData(index, combo->currentIndex());
        }
        break;
    default:
        QStyledItemDelegate::setModelData(editor, model, index);
    }
}


ManipulatorVariableListViewBase::ManipulatorVariableListViewBase()
{
    impl = new Impl(this);
}


ManipulatorVariableListViewBase::Impl::Impl(ManipulatorVariableListViewBase* self)
    : self(self),
      targetItemPicker(self)
{
    self->setDefaultLayoutArea(View::RIGHT);

    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);

    int hs = self->style()->pixelMetric(QStyle::PM_LayoutHorizontalSpacing);
    
    auto hbox = new QHBoxLayout;
    hbox->setSpacing(0);
    hbox->addSpacing(hs);
    targetLabel.setStyleSheet("font-weight: bold");
    hbox->addWidget(&targetLabel, 0, Qt::AlignVCenter);
    hbox->addSpacing(hs);
    addButton.setText(_("Add"));
    addButton.sigClicked().connect([&](){ addVariableIntoCurrentIndex(false); });
    hbox->addWidget(&addButton);
    hbox->addStretch();
    vbox->addLayout(hbox);

    // Setup the table
    auto hframe = new QFrame;
    hframe->setFrameStyle(QFrame::HLine | QFrame::Sunken);
    vbox->addWidget(hframe);
    setFrameShape(QFrame::NoFrame);
    setSelectionBehavior(QAbstractItemView::SelectRows);
    setSelectionMode(QAbstractItemView::ExtendedSelection);
    setTabKeyNavigation(true);
    setCornerButtonEnabled(true);
    setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    setItemDelegate(new CustomizedItemDelegate(this));
    setEditTriggers(
        /* QAbstractItemView::CurrentChanged | */
        QAbstractItemView::DoubleClicked |
        /* QAbstractItemView::SelectedClicked | */
        QAbstractItemView::EditKeyPressed |
        QAbstractItemView::AnyKeyPressed);

    variableListModel = new VariableListModel(this);
    setModel(variableListModel);

    auto hheader = horizontalHeader();
    hheader->setSectionResizeMode(IdColumn, QHeaderView::ResizeToContents);
    hheader->setSectionResizeMode(ValueTypeColumn, QHeaderView::ResizeToContents);
    hheader->setSectionResizeMode(ValueColumn, QHeaderView::ResizeToContents);
    hheader->setSectionResizeMode(NoteColumn, QHeaderView::Stretch);
    auto vheader = verticalHeader();
    vheader->setSectionResizeMode(QHeaderView::ResizeToContents);
    vheader->hide();

    vbox->addWidget(this);
    self->setLayout(vbox);

    targetItemPicker.sigTargetItemChanged().connect(
        [&](ManipulatorVariableListItemBase* item){
            setManipulatorVariableListItem(item); });
}


ManipulatorVariableListViewBase::~ManipulatorVariableListViewBase()
{
    delete impl;
}


void ManipulatorVariableListViewBase::Impl::setManipulatorVariableListItem(ManipulatorVariableListItemBase* item)
{
    targetItem = item;

    if(item){
        targetLabel.setText(item->name().c_str());
        variables = item->variableList();
        variableListModel->setVariableList(variables);
    } else {
        targetLabel.setText("---");
        variables = nullptr;
        variableListModel->setVariableList(nullptr);
    }
    addButton.setEnabled(targetItem != nullptr);
}


void ManipulatorVariableListViewBase::Impl::addVariableIntoCurrentIndex(bool doInsert)
{
    auto current = selectionModel()->currentIndex();
    int row = current.isValid() ? current.row() : variableListModel->numVariables();
    addVariable(row, doInsert);
}


void ManipulatorVariableListViewBase::Impl::addVariable(int row, bool doInsert)
{
    if(variables){
        auto id = variables->createNextId();
        ManipulatorVariablePtr variable = new ManipulatorVariable(id);
        variable->setValue(0);
        variableListModel->addVariable(row, variable, doInsert);
        resizeColumnToContents(IdColumn);
        //resizeColumnToContents(ValueColumn);
    }
}


void ManipulatorVariableListViewBase::Impl::removeSelectedVariables()
{
    variableListModel->removeVariables(selectionModel()->selectedRows());
}


void ManipulatorVariableListViewBase::Impl::keyPressEvent(QKeyEvent* event)
{
    bool processed = true;

    switch(event->key()){
    case Qt::Key_Escape:
        clearSelection();
        break;
    case Qt::Key_Insert:
        addVariableIntoCurrentIndex(true);
        break;
    case Qt::Key_Delete:
        removeSelectedVariables();
        break;
    default:
        processed = false;
        break;
    }
        
    if(!processed && (event->modifiers() & Qt::ControlModifier)){
        processed = true;
        switch(event->key()){
        case Qt::Key_A:
            selectAll();
            break;
        default:
            processed = false;
            break;
        }
    }

    if(!processed){
        QTableView::keyPressEvent(event);
    }
}

       
void ManipulatorVariableListViewBase::Impl::mousePressEvent(QMouseEvent* event)
{
    QTableView::mousePressEvent(event);

    if(event->button() == Qt::RightButton){
        int row = rowAt(event->pos().y());
        if(row >= 0){
            showContextMenu(row, event->globalPos());
        }
    }
}


void ManipulatorVariableListViewBase::Impl::showContextMenu(int row, QPoint globalPos)
{
    contextMenuManager.setNewPopupMenu(this);

    contextMenuManager.addItem(_("Add"))
        ->sigTriggered().connect([=](){ addVariable(row, false); });

    if(variableListModel->numVariables() > 0){
        contextMenuManager.addItem(_("Remove"))
            ->sigTriggered().connect([=](){ removeSelectedVariables(); });
    }
    
    contextMenuManager.popupMenu()->popup(globalPos);
}


bool ManipulatorVariableListViewBase::storeState(Archive& archive)
{
    impl->targetItemPicker.storeTargetItem(archive, "currentItem");
    return true;
}


bool ManipulatorVariableListViewBase::restoreState(const Archive& archive)
{
    impl->targetItemPicker.restoreTargetItemLater(archive, "currentItem");
    return true;
}
