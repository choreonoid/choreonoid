#include "MprVariableListView.h"
#include "MprMultiVariableListItem.h"
#include "MprControllerItemBase.h"
#include <cnoid/MprVariableList>
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/TargetItemPicker>
#include <cnoid/ConnectionSet>
#include <cnoid/Buttons>
#include <cnoid/ButtonGroup>
#include <QBoxLayout>
#include <QLabel>
#include <QTableView>
#include <QHeaderView>
#include <QAbstractTableModel>
#include <QStyledItemDelegate>
#include <QComboBox>
#include <QSpinBox>
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
    MprVariableListPtr variableList;
    ScopedConnectionSet variableListConnections;
    
    VariableListModel(QObject* parent);
    void setVariableList(MprVariableList* variables);
    void refresh();
    int numVariables() const;
    MprVariable* variableAtRow(int row);
    int rowOfVariable(MprVariable* variable) const;
    virtual int rowCount(const QModelIndex& parent) const override;
    virtual int columnCount(const QModelIndex& parent) const override;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    virtual QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    virtual Qt::ItemFlags flags(const QModelIndex& index) const override;
    virtual QVariant data(const QModelIndex& index, int role) const override;
    virtual bool setData(const QModelIndex& index, const QVariant& value, int role) override;
    void removeVariables(QModelIndexList selected);
    void onVariableAdded(int variableIndex);
    void onVariableRemoved(int variableIndex);
    void onVariableUpdated(int variableIndex, int flags);
};

class CustomizedItemDelegate : public QStyledItemDelegate
{
public:
    MprVariableListView::Impl* view;
    
    CustomizedItemDelegate(MprVariableListView::Impl* view);
    virtual QWidget* createEditor(
        QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    QWidget* createVariableValueEditor(MprVariable* variable, QWidget* parent) const;
    virtual void setEditorData(QWidget* editor, const QModelIndex& index) const override;
    virtual void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;
};

/**
   Spin box that can input both the integer number and string
*/
class IdSpinBox : public QSpinBox
{
    mutable QString inputText;
    mutable QString finalText;
    bool isStringIdEnabled;
public:
    IdSpinBox(bool isStringIdEnabled, QWidget* parent)
        : QSpinBox(parent), isStringIdEnabled(isStringIdEnabled) { }

    virtual QValidator::State validate(QString& input, int& pos) const override {
        auto state = QSpinBox::validate(input, pos);
        if(isStringIdEnabled){
            if(state == QValidator::Invalid){
                inputText = input;
                state = QValidator::Intermediate;
            } else {
                inputText.clear();
            }
        }
        return state;
    }
    virtual void fixup(QString &input) const override {
        if(isStringIdEnabled){
            finalText = inputText;
        }
        QSpinBox::fixup(input);
    }
      
    QVariant getId(){
        if(finalText.isEmpty()){
            return value();
        }
        bool ok;
        int id = finalText.toInt(&ok);
        if(ok){
            return id;
        }
        return finalText;
    }
};

}

namespace cnoid {

class MprVariableListView::Impl : public QTableView
{
public:
    MprVariableListView* self;
    TargetItemPicker<MprMultiVariableListItem> targetItemPicker;
    MprMultiVariableListItemPtr targetMultiVariableListItem;
    MprVariableListPtr currentVariableList;
    int lastVariableListIndex;
    VariableListModel* variableListModel;
    int hSpacing;
    QLabel targetLabel;
    QHBoxLayout listTypeBox;
    ButtonGroup listTypeGroup;
    vector<RadioButton*> listTypeRadios;
    PushButton addButton;
    MenuManager contextMenuManager;

    Impl(MprVariableListView* self);
    void setTargetMultiVariableListItem(MprMultiVariableListItem* item);
    void updateTypeSelectionRadioButtons();
    void setCurrentVariableList(int listIndex);
    void addVariableIntoCurrentIndex(bool doInsert);
    void addVariable(int row, bool doInsert);
    void removeSelectedVariables();
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    void showContextMenu(int row, QPoint globalPos);
};

}


namespace {

VariableListModel::VariableListModel(QObject* parent)
    : QAbstractTableModel(parent)
{

}


void VariableListModel::setVariableList(MprVariableList* variableList)
{
    beginResetModel();
    
    this->variableList = variableList;

    variableListConnections.disconnect();
    if(variableList){
        variableListConnections.add(
            variableList->sigVariableAdded().connect(
                [&](int index){ onVariableAdded(index); }));
        variableListConnections.add(
            variableList->sigVariableRemoved().connect(
                [&](int index, MprVariable*){ onVariableRemoved(index); }));
        variableListConnections.add(
            variableList->sigVariableUpdated().connect(
                [&](int index, int flags){ onVariableUpdated(index, flags); }));
    }
    
    endResetModel();
}


void VariableListModel::refresh()
{
    beginResetModel();
    endResetModel();
}    


int VariableListModel::numVariables() const
{
    return variableList ?  variableList->numVariables() : 0;
}


MprVariable* VariableListModel::variableAtRow(int row)
{
    return variableList->variableAt(row);
}


int VariableListModel::rowOfVariable(MprVariable* variable) const
{
    if(variableList){
        return variableList->indexOf(variable);
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
                if(variableList){
                    if(variableList->isNumberIdEnabled()){
                        if(!variableList->isStringIdEnabled()){
                            return _("No.");
                        }
                    } else {
                        return _("Name");
                    }
                }
                return _("ID");
                break;
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
    if(!variableList || parent.isValid()){
        return QModelIndex();
    }
    if(row >= 0 && row < numVariables()){
        auto variable = const_cast<VariableListModel*>(this)->variableAtRow(row);
        return createIndex(row, column, variable);
    }
    return QModelIndex();
}
    

Qt::ItemFlags VariableListModel::flags(const QModelIndex& index) const
{
    if(index.column() == ValueTypeColumn && variableList
       && variableList->isGeneralVariableValueTypeUnchangeable()){
        return QAbstractTableModel::flags(index); // Non-editable
    }
    return QAbstractTableModel::flags(index) | Qt::ItemIsEditable;
}


QVariant VariableListModel::data(const QModelIndex& index, int role) const
{
    auto variable = static_cast<MprVariable*>(index.internalPointer());

    if(!variable || !index.isValid()){
        return QVariant();
    }

    int column = index.column();
    if(role == Qt::DisplayRole || role == Qt::EditRole){
        if(column == IdColumn){
            return variable->id().label().c_str();

        } else if(column == ValueTypeColumn){
            switch(variable->valueType()){
            case MprVariable::Int:
                return _("Integer");
            case MprVariable::Double:
                return _("Real");
            case MprVariable::Bool:
                return _("Logical");
            case MprVariable::String:
                return _("String");
            default:
                return _("Unknown");
            }

        } else if(column == ValueColumn){
            switch(variable->valueType()){
            case MprVariable::Int:
                return variable->intValue();
            case MprVariable::Double:
                return QString("%1").arg(variable->doubleValue(), 0, 'g');
            case MprVariable::Bool:
                return variable->boolValue() ? _("True") : _("False");
            case MprVariable::String:
                return variable->stringValue().c_str();
            default:
                break;
            }

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
        auto variable = static_cast<MprVariable*>(index.internalPointer());
        int column = index.column();

        if(column == IdColumn){
            bool ok;
            GeneralId newId = value.toInt(&ok);
            if(!ok){
                newId = value.toString().toStdString();
            }
            if(newId != variable->id()){
                if(newId.isInt() && newId.toInt() < 0){
                    showWarningDialog(_("Negative number cannot be used as ID."));
                } else {
                    if(variableList->resetId(variable, newId)){
                        Q_EMIT dataChanged(index, index, {role});
                    } else {
                        showWarningDialog(
                            format(_("ID {0} is already in use."), newId.label()));
                    }
                }
            }

        } else if(column == ValueTypeColumn){
            if(variable->changeValueType(MprVariable::TypeId(value.toInt()))){
                Q_EMIT dataChanged(index, index, {role});
            }

        } else if(column == ValueColumn){
            switch(variable->valueType()){
            case MprVariable::Int:
                variable->setValue(value.toInt());
                break;
            case MprVariable::Double:
                variable->setValue(value.toDouble());
                break;
            case MprVariable::Bool:
                variable->setValue(value.toBool());
                break;
            case MprVariable::String:
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


void VariableListModel::removeVariables(QModelIndexList selected)
{
    if(variableList){
        std::sort(selected.begin(), selected.end());
        int numRemoved = 0;
        for(auto& index : selected){
            int variableIndex = index.row() - numRemoved;
            variableList->removeAt(variableIndex);
            ++numRemoved;
        }
    }
}


void VariableListModel::onVariableAdded(int variableIndex)
{
    if(numVariables() == 0){
        // Remove the empty row first
        beginRemoveRows(QModelIndex(), 0, 0);
        endRemoveRows();
    }
    beginInsertRows(QModelIndex(), variableIndex, variableIndex);
    endInsertRows();
}


void VariableListModel::onVariableRemoved(int variableIndex)
{
    beginRemoveRows(QModelIndex(), variableIndex, variableIndex);
    endRemoveRows();
    if(numVariables() == 0){
        // This is necessary to show the empty row
        beginResetModel();
        endResetModel();
    }
}


void VariableListModel::onVariableUpdated(int variableIndex, int flags)
{
    if(flags & MprVariable::IdUpdate){
        auto modelIndex = index(variableIndex, IdColumn, QModelIndex());
        Q_EMIT dataChanged(modelIndex, modelIndex, { Qt::EditRole });
    }
    if(flags & MprVariable::ValueUpdate){
        auto modelIndex1 = index(variableIndex, ValueTypeColumn, QModelIndex());
        auto modelIndex2 = index(variableIndex, ValueColumn, QModelIndex());
        Q_EMIT dataChanged(modelIndex1, modelIndex2, { Qt::EditRole });
    }
    if(flags & MprVariable::NoteUpdate){
        auto modelIndex = index(variableIndex, NoteColumn, QModelIndex());
        Q_EMIT dataChanged(modelIndex, modelIndex, { Qt::EditRole });
    }
}
        

CustomizedItemDelegate::CustomizedItemDelegate(MprVariableListView::Impl* view)
    : QStyledItemDelegate(view),
      view(view)
{

}


QWidget* CustomizedItemDelegate::createEditor
(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    QWidget* editor = nullptr;
    
    auto variable = static_cast<MprVariable*>(index.internalPointer());
    
    switch(index.column()){

    case IdColumn:
    {
        auto& id = variable->id();
        if(id.isInt()){
            auto spin = new IdSpinBox(view->currentVariableList->isStringIdEnabled(), parent);
            spin->setFrame(false);
            spin->setRange(0, 9999);
            spin->setValue(id.toInt());
            editor = spin;
        } else {
            editor = QStyledItemDelegate::createEditor(parent, option, index);
        }
        break;
    }
    case ValueTypeColumn:
    {
        auto combo = new QComboBox(parent);
        combo->addItem(_("Integer"), MprVariable::Int);
        combo->addItem(_("Real"), MprVariable::Double);
        combo->addItem(_("Logical"), MprVariable::Bool);
        combo->addItem(_("String"), MprVariable::String);
        combo->setCurrentIndex(variable->valueType());
        editor = combo;
        break;
    }
    case ValueColumn:
        editor = createVariableValueEditor(variable, parent);
        break;
        
    default:
        break;
    }

    if(!editor){
        editor = QStyledItemDelegate::createEditor(parent, option, index);
    }

    return editor;
}


QWidget* CustomizedItemDelegate::createVariableValueEditor(MprVariable* variable, QWidget* parent) const
{
    switch(variable->valueType()){
    case MprVariable::Int:
    {
        auto spin = new QSpinBox(parent);
        spin->setRange(-999999, 999999);
        spin->setValue(variable->intValue());
        return spin;
        break;
    }
    case MprVariable::Double:
    {
        auto spin = new QDoubleSpinBox(parent);
        spin->setDecimals(3);
        spin->setRange(-999999.0, 999999.0);
        spin->setValue(variable->doubleValue());
        return spin;
        break;
    }
    case MprVariable::Bool:
    {
        auto combo = new QComboBox(parent);
        combo->addItem(_("True"));
        combo->addItem(_("False"));
        combo->setCurrentIndex(variable->boolValue() ? 0 : 1);
        return combo;
    }
    default:
        break;
    }

    return nullptr;
}
            

void CustomizedItemDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
    QStyledItemDelegate::setEditorData(editor, index);
}


void CustomizedItemDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    switch(index.column()){
    case IdColumn:
        if(auto spin = dynamic_cast<IdSpinBox*>(editor)){
            model->setData(index, spin->getId());
        } else {
            QStyledItemDelegate::setModelData(editor, model, index);
        }
        break;
    case ValueTypeColumn:
        if(auto combo = dynamic_cast<QComboBox*>(editor)){
            model->setData(index, combo->currentData().toInt());
        }
        break;
    case ValueColumn:
        if(auto combo = dynamic_cast<QComboBox*>(editor)){
            model->setData(index, (combo->currentIndex() == 0) ? true : false);
        } else {
            QStyledItemDelegate::setModelData(editor, model, index);
        }
        break;
    default:
        QStyledItemDelegate::setModelData(editor, model, index);
    }
}

}


void MprVariableListView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<MprVariableListView>(
        "MprVariableListView", N_("Variables"), ViewManager::SINGLE_OPTIONAL);
}


MprVariableListView::MprVariableListView()
{
    impl = new Impl(this);
}


MprVariableListView::Impl::Impl(MprVariableListView* self)
    : self(self),
      targetItemPicker(self)
{
    self->setDefaultLayoutArea(View::BOTTOM);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    hSpacing = self->style()->pixelMetric(QStyle::PM_LayoutHorizontalSpacing);

    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);

    auto hbox = new QHBoxLayout;
    hbox->setSpacing(0);
    hbox->addSpacing(hSpacing );
    targetLabel.setStyleSheet("font-weight: bold");
    hbox->addWidget(&targetLabel, 0, Qt::AlignVCenter);
    hbox->addSpacing(hSpacing * 2);

    hbox->addLayout(&listTypeBox);
    listTypeGroup.sigButtonToggled().connect(
        [&](int id, bool checked){
            if(checked){ setCurrentVariableList(id); }
        });
    hbox->addSpacing(hSpacing);
    hbox->addStretch();
    
    addButton.setText(_("Add"));
    addButton.sigClicked().connect([&](){ addVariableIntoCurrentIndex(false); });
    hbox->addWidget(&addButton);
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
    setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
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
        [&](MprMultiVariableListItem* item){ setTargetMultiVariableListItem(item); });

    lastVariableListIndex = 0;
}


MprVariableListView::~MprVariableListView()
{
    delete impl;
}


void MprVariableListView::onAttachedMenuRequest(MenuManager& menuManager)
{
    menuManager.addItem(_("Refresh"))->sigTriggered().connect(
        [&](){ impl->variableListModel->refresh(); });
}


void MprVariableListView::Impl::setTargetMultiVariableListItem(MprMultiVariableListItem* item)
{
    targetMultiVariableListItem = item;
    currentVariableList.reset();

    if(item){
        if(auto controller = item->findOwnerItem<MprControllerItemBase>()){
            targetLabel.setText(
                format("{0} - {1}", controller->displayName(), item->displayName()).c_str());
        } else {
            targetLabel.setText(item->displayName().c_str());
        }
    } else {
        targetLabel.setText("---");
    }

    updateTypeSelectionRadioButtons();
    setCurrentVariableList(lastVariableListIndex);
    
    addButton.setEnabled(currentVariableList != nullptr);
}


void MprVariableListView::Impl::updateTypeSelectionRadioButtons()
{
    while(listTypeBox.count()){
        listTypeBox.takeAt(0);
    }
    for(auto& radio : listTypeRadios){
        listTypeGroup.removeButton(radio);
        delete radio;
    }
    listTypeRadios.clear();

    auto& target = targetMultiVariableListItem;
    if(!target){
        return;
    }
    int n = target->numVariableLists();
    for(int i=0; i < n; ++i){
        if(auto list = target->variableListAt(i)){
            QString caption;
            switch(list->variableType()){
            case MprVariableList::GeneralVariable:
                caption = _("General");
                break;
            case MprVariableList::IntVariable:
                caption = _("Integer");
                break;
            case MprVariableList::DoubleVariable:
                caption = _("Real");
                break;
            case MprVariableList::BoolVariable:
                caption = _("Boolean");
                break;
            case MprVariableList::StringVariable:
                caption = _("String");
                break;
            default:
                continue;
            }
            auto radio = new RadioButton(caption);
            listTypeGroup.addButton(radio, i);
            listTypeBox.addWidget(radio);
            listTypeBox.addSpacing(hSpacing / 2);
            listTypeRadios.push_back(radio);
        }
    }
}


void MprVariableListView::Impl::setCurrentVariableList(int listIndex)
{
    currentVariableList.reset();
    
    auto& target = targetMultiVariableListItem;
    bool hideValueTypeColumn = true;

    if(target && target->numVariableLists() > 0){
        int n = target->numVariableLists();
        if(listIndex >= n){
            listIndex = n - 1;
        }
        currentVariableList = target->variableListAt(listIndex);
        lastVariableListIndex = listIndex;

        if(currentVariableList->variableType() == MprVariableList::GeneralVariable){
            hideValueTypeColumn = false;
        }
        if(auto button = listTypeGroup.button(listIndex)){
            listTypeGroup.blockSignals(true);
            button->setChecked(true);
            listTypeGroup.blockSignals(false);
        }
    }

    setColumnHidden(ValueTypeColumn, hideValueTypeColumn);
    variableListModel->setVariableList(currentVariableList);
}


void MprVariableListView::Impl::addVariableIntoCurrentIndex(bool doInsert)
{
    auto current = selectionModel()->currentIndex();
    int row = current.isValid() ? current.row() : variableListModel->numVariables();
    addVariable(row, doInsert);
}


void MprVariableListView::Impl::addVariable(int row, bool doInsert)
{
    if(currentVariableList){
        auto id = currentVariableList->createNextId();
        MprVariablePtr variable = new MprVariable(id, currentVariableList->defaultValue());
        int newVariableIndex = doInsert ? row : row + 1;
        currentVariableList->insert(newVariableIndex, variable);
        resizeColumnToContents(IdColumn);
        //resizeColumnToContents(ValueColumn);
    }
}


void MprVariableListView::Impl::removeSelectedVariables()
{
    variableListModel->removeVariables(selectionModel()->selectedRows());
}


void MprVariableListView::Impl::keyPressEvent(QKeyEvent* event)
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

       
void MprVariableListView::Impl::mousePressEvent(QMouseEvent* event)
{
    QTableView::mousePressEvent(event);

    if(event->button() == Qt::RightButton){
        int row = rowAt(event->pos().y());
        if(row >= 0){
            showContextMenu(row, event->globalPos());
        }
    }
}


void MprVariableListView::Impl::showContextMenu(int row, QPoint globalPos)
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


bool MprVariableListView::storeState(Archive& archive)
{
    impl->targetItemPicker.storeTargetItem(archive, "current_item");
    archive.write("variable_list_index", impl->lastVariableListIndex);
    return true;
}


bool MprVariableListView::restoreState(const Archive& archive)
{
    impl->targetItemPicker.restoreTargetItemLater(archive, "current_item");
    archive.read("variable_list_index", impl->lastVariableListIndex);
    return true;
}
