#include "ManipulatorProgramViewBase.h"
#include "ManipulatorProgramItemBase.h"
#include "ManipulatorProgram.h"
#include "ManipulatorStatements.h"
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/TargetItemPicker>
#include <cnoid/TreeWidget>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include <cnoid/BodyItem>
#include <cnoid/Buttons>
#include <cnoid/StringListComboBox>
#include <QBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QProxyStyle>
#include <QStyledItemDelegate>
#include <QPainter>
#include <QLineEdit>
#include <QItemEditorFactory>
#include <QStandardItemEditorCreator>
#include <unordered_map>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

static constexpr int NumColumns = 4;

class StatementItem : public QTreeWidgetItem
{
    ManipulatorStatementPtr statement_;
public:
    ManipulatorProgramViewBase::Impl* viewImpl;
    ManipulatorProgramViewBase::StatementDelegate* delegate;

    StatementItem(ManipulatorStatement* statement, ManipulatorProgramViewBase::Impl* viewImpl);
    ~StatementItem();
    virtual QVariant data(int column, int role) const override;
    virtual void setData(int column, int role, const QVariant& value) override;
    StatementItem* getParentStatementItem() const { return static_cast<StatementItem*>(parent()); }
    StructuredStatement* getParentStatement() const;
    ManipulatorProgram* getProgram() const;
    template<class StatementType>
    ref_ptr<StatementType> statement() const { return dynamic_pointer_cast<StatementType>(statement_); }
    ManipulatorStatementPtr statement() const { return statement_; }
};


class ProgramViewDelegate : public QStyledItemDelegate
{
    ManipulatorProgramViewBase::Impl* viewImpl;
public:
    ProgramViewDelegate(ManipulatorProgramViewBase::Impl* viewImpl);
    virtual void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
    virtual QWidget* createEditor(
        QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual void updateEditorGeometry(
        QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual void setEditorData(QWidget* editor, const QModelIndex& index) const override;
    virtual void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;
};


/**
   The style to draw the drop line over all the columns.
   This style is currently not used because it is difficult to make the indent
   correspoinding to the drop position.
*/
class TreeWidgetStyle : public QProxyStyle
{
    ManipulatorProgramViewBase::Impl* viewImpl;
public:
    TreeWidgetStyle(ManipulatorProgramViewBase::Impl* viewImpl);
    void drawPrimitive(
        PrimitiveElement element, const QStyleOption* option, QPainter* painter, const QWidget* widget = 0) const;
};


class ScopedCounter
{
    int& counter;
public:
    ScopedCounter(int& counter) : counter(counter){
        ++counter;
    }
    ~ScopedCounter(){
        --counter;
    }
};

}

namespace cnoid {

class ManipulatorProgramViewBase::StatementDelegate::Impl
{
public:
    ManipulatorProgramViewBase::StatementDelegate* self;
    ProgramViewDelegate* mainDelegate;
    const StatementItem* currentItem;
    QWidget* currentParentWidget;
    const QStyleOptionViewItem* pCurrentOption;
    QAbstractItemModel* currentModel;
    const QModelIndex* pCurrentModelIndex;

    Impl(ManipulatorProgramViewBase::StatementDelegate* self);
    void setView(ManipulatorProgramViewBase::Impl* viewImpl);
    int actualLabelSpan(ManipulatorStatement* statement, int column) const;
};

class ManipulatorProgramViewBase::Impl : public TreeWidget
{
public:
    ManipulatorProgramViewBase* self;
    TargetItemPicker<ManipulatorProgramItemBase> targetItemPicker;
    ManipulatorProgramItemBasePtr programItem;
    unordered_map<ManipulatorStatementPtr, StatementItem*> statementItemMap;
    int statementItemOperationCallCounter;
    ScopedConnectionSet programConnections;
    ManipulatorStatementPtr currentStatement;
    Signal<void(ManipulatorStatement* statement)> sigCurrentStatementChanged;
    ManipulatorStatementPtr prevCurrentStatement;
    vector<ManipulatorStatementPtr> statementsToPaste;
    ProgramViewDelegate* mainDelegate;
    ref_ptr<StatementDelegate> defaultStatementDelegate;
    unordered_map<type_index, ref_ptr<StatementDelegate>> statementDelegateMap;
    QLabel programNameLabel;
    ToolButton optionMenuButton;
    MenuManager optionMenuManager;
    MenuManager contextMenuManager;
    QHBoxLayout buttonBox[2];

    Impl(ManipulatorProgramViewBase* self);
    ~Impl();
    void setupWidgets();
    StatementDelegate* findStatementDelegate(ManipulatorStatement* statement);
    void onOptionMenuClicked();
    ScopedCounter scopedCounterOfStatementItemOperationCall();
    bool isDoingStatementItemOperation() const;
    void setProgramItem(ManipulatorProgramItemBase* item);
    void updateStatementTree();
    void addProgramStatementsToTree(ManipulatorProgram* program, QTreeWidgetItem* parentItem);
    void onCurrentTreeWidgetItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous);
    void setCurrentStatement(ManipulatorStatement* statement);
    void onTreeWidgetItemClicked(QTreeWidgetItem* item, int /* column */);
    StatementItem* statementItemFromStatement(ManipulatorStatement* statement);
    bool insertStatement(ManipulatorStatement* statement, int insertionType);
    void onStatementInserted(ManipulatorProgram* program, ManipulatorProgram::iterator iter);
    void onStatementRemoved(ManipulatorProgram* program, ManipulatorStatement* statement);
    void forEachStatementInTreeEditEvent(
        const QModelIndex& parent, int start, int end,
        function<void(StructuredStatement* parentStatement, ManipulatorProgram* program,
                      int index, ManipulatorStatement* statement)> func);
    void onRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end);
    void onRowsRemoved(const QModelIndex& parent, int start, int end);
    void onRowsInserted(const QModelIndex& parent, int start, int end);
    bool removeDummyStatementAroundInsertionPosition(
        StructuredStatement* parentStatement, ManipulatorProgram* program, int index, int direction);
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    void showContextMenu(QPoint globalPos);
    void copySelectedStatements(bool doCut);
    void pasteStatements();

    QModelIndex indexFromItem(QTreeWidgetItem* item, int column = 0) const {
        return TreeWidget::indexFromItem(item, column);
    }
    StatementItem* itemFromIndex(const QModelIndex& index) const {
        return static_cast<StatementItem*>(TreeWidget::itemFromIndex(index));
    }
};

}


StatementItem::StatementItem(ManipulatorStatement* statement_, ManipulatorProgramViewBase::Impl* viewImpl)
    : statement_(statement_),
      viewImpl(viewImpl)
{
    viewImpl->statementItemMap[statement_] = this;
    delegate = viewImpl->findStatementDelegate(statement_);
    
    Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsDragEnabled | Qt::ItemIsEditable;
    if(statement<StructuredStatement>()){
        flags |= Qt::ItemIsDropEnabled;
    }
    setFlags(flags);
}


StatementItem::~StatementItem()
{
    viewImpl->statementItemMap.erase(statement());
}


QVariant StatementItem::data(int column, int role) const
{
    if(role == Qt::DisplayRole){
        int span = delegate->labelSpan(statement(), column);
        if(span == 1){
            return QString(statement()->label(column).c_str());
        } else {
            return QVariant();
        }
    } else if(role == Qt::EditRole){
        delegate->impl->currentItem = this;
        return delegate->dataOfEditRole(statement(), column);
    }
    return QTreeWidgetItem::data(column, role);
}


void StatementItem::setData(int column, int role, const QVariant& value)
{
    if(role == Qt::EditRole){
        delegate->setDataOfEditRole(statement(), column, value);
    }
    QTreeWidgetItem::setData(column, role, value);
}


StructuredStatement* StatementItem::getParentStatement() const
{
    if(auto parentStatementItem = getParentStatementItem()){
        return parentStatementItem->statement<StructuredStatement>();
    }
    return nullptr;
}


ManipulatorProgram* StatementItem::getProgram() const
{
    if(auto parentStatement = getParentStatement()){
        return parentStatement->lowerLevelProgram();
    }
    if(viewImpl->programItem){
        return viewImpl->programItem->program();
    }
    return nullptr;
}


ProgramViewDelegate::ProgramViewDelegate(ManipulatorProgramViewBase::Impl* viewImpl)
    : viewImpl(viewImpl)
{

}


void ProgramViewDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    auto item = viewImpl->itemFromIndex(index);
    auto statement = item->statement();
    
    int column = index.column();
    int span = item->delegate->impl->actualLabelSpan(statement, column);
    if(span == 1){
        QStyledItemDelegate::paint(painter, option, index);
    } else if(span > 1){
        auto rect = viewImpl->visualRect(index);
        for(int i=1; i < span; ++i){
            auto rect2 = viewImpl->visualRect(viewImpl->indexFromItem(item, column + i));
            rect = rect.united(rect2);
        }
        painter->save();
        if(option.state & QStyle::State_Selected){
            painter->fillRect(rect, option.palette.highlight());
            painter->setPen(option.palette.highlightedText().color());
        }
        painter->drawText(rect, 0, statement->label(column).c_str());
        painter->restore();
    }
}


QWidget* ProgramViewDelegate::createEditor
(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    auto item = viewImpl->itemFromIndex(index);
    auto statement = item->statement();
    int column = index.column();
    int span = item->delegate->labelSpan(statement, column);
    if(span == 0){
        return nullptr;
    } else if(column == 0 && span == 1){
        return nullptr;
    }

    auto delegate = item->delegate;
    delegate->impl->currentParentWidget = parent;
    delegate->impl->pCurrentOption = &option;
    delegate->impl->pCurrentModelIndex = &index;
    return delegate->createEditor(statement, column);
}


void ProgramViewDelegate::updateEditorGeometry
(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    auto item = viewImpl->itemFromIndex(index);
    auto statement = item->statement();
    int column = index.column();
    int span = item->delegate->impl->actualLabelSpan(statement, column);
    QRect rect;
    if(span == 0){
        int topColumn = column - 1;
        while(topColumn >= 0){
            if(item->delegate->impl->actualLabelSpan(statement, topColumn) > 0){
                break;
            }
            if(topColumn > 0){
                --topColumn;
            } else {
                break;
            }
        }
        for(int i=topColumn; i < NumColumns; ++i){
            auto rect2 = viewImpl->visualRect(viewImpl->indexFromItem(item, i));
            rect = rect.united(rect2);
        }
    } else if(span == 1){
        rect = option.rect;
    } else {
        rect = viewImpl->visualRect(index);
        for(int i=1; i < span; ++i){
            auto rect2 = viewImpl->visualRect(viewImpl->indexFromItem(item, column + i));
            rect = rect.united(rect2);
        }
    }
    editor->setGeometry(rect);
}


void ProgramViewDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
    auto item = viewImpl->itemFromIndex(index);
    auto statement = item->statement();
    item->delegate->impl->pCurrentModelIndex = &index;
    item->delegate->setEditorData(statement, index.column(), editor);
}


void ProgramViewDelegate::setModelData
(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    auto item = viewImpl->itemFromIndex(index);
    auto statement = item->statement();
    item->delegate->impl->currentModel = model;
    item->delegate->impl->pCurrentModelIndex = &index;
    item->delegate->setStatementData(statement, index.column(), editor);
}


TreeWidgetStyle::TreeWidgetStyle(ManipulatorProgramViewBase::Impl* viewImpl)
    : QProxyStyle(viewImpl->style()),
      viewImpl(viewImpl)
{

}


void TreeWidgetStyle::drawPrimitive
(PrimitiveElement element, const QStyleOption* option, QPainter* painter, const QWidget* widget) const
{
    if(element == QStyle::PE_IndicatorItemViewItemDrop && !option->rect.isNull()){
        QStyleOption opt(*option);
        auto item = dynamic_cast<StatementItem*>(viewImpl->itemAt(opt.rect.center()));
        opt.rect.setLeft(0);
        if(widget){
            opt.rect.setRight(widget->width());
        }
        QProxyStyle::drawPrimitive(element, &opt, painter, widget);
        return;
    }
    QProxyStyle::drawPrimitive(element, option, painter, widget);
}


ManipulatorProgramViewBase::StatementDelegate::StatementDelegate()
{
    impl = new Impl(this);
}


ManipulatorProgramViewBase::StatementDelegate::Impl::Impl
(ManipulatorProgramViewBase::StatementDelegate* self)
    : self(self)
{
    mainDelegate = nullptr;
    currentItem = nullptr;
    currentParentWidget = nullptr;
    pCurrentOption = nullptr;
    currentModel = nullptr;
    pCurrentModelIndex = nullptr;
}


ManipulatorProgramViewBase::StatementDelegate::~StatementDelegate()
{
    delete impl;
}


void ManipulatorProgramViewBase::StatementDelegate::Impl::setView
(ManipulatorProgramViewBase::Impl* viewImpl)
{
    mainDelegate = viewImpl->mainDelegate;
}


int ManipulatorProgramViewBase::StatementDelegate::Impl::actualLabelSpan
(ManipulatorStatement* statement, int column) const
{
    int span = self->labelSpan(statement, column);
    if(span == SpanToLast){
        span = (NumColumns - column);
    }
    return span;
}


int ManipulatorProgramViewBase::StatementDelegate::labelSpan
(ManipulatorStatement* /* statement */, int /* column */) const
{
    return 1;
}


QVariant ManipulatorProgramViewBase::StatementDelegate::dataOfEditRole
(ManipulatorStatement* statement, int column) const
{
    int span = labelSpan(statement, column);
    if(span == 1){
        return QString(statement->label(column).c_str());
    }
    return QVariant();
}


void ManipulatorProgramViewBase::StatementDelegate::setDataOfEditRole
(ManipulatorStatement* /* statement */, int /* column */, const QVariant& /* value */) const
{

}


QWidget* ManipulatorProgramViewBase::StatementDelegate::createEditor
(ManipulatorStatement* /* statement */, int /* column */) const
{
    return nullptr;
}


QWidget* ManipulatorProgramViewBase::StatementDelegate::createDefaultEditor() const
{
    return impl->mainDelegate->QStyledItemDelegate::createEditor(
        impl->currentParentWidget, *impl->pCurrentOption, *impl->pCurrentModelIndex);
}


void ManipulatorProgramViewBase::StatementDelegate::setEditorData
(ManipulatorStatement* /* statement */, int /* column */, QWidget* editor) const
{
    impl->mainDelegate->QStyledItemDelegate::setEditorData(editor, *impl->pCurrentModelIndex);
}


void ManipulatorProgramViewBase::StatementDelegate::setStatementData
(ManipulatorStatement* /* statement */, int /* column */, QWidget* editor ) const
{
    impl->mainDelegate->QStyledItemDelegate::setModelData(editor, impl->currentModel, *impl->pCurrentModelIndex);
}


ManipulatorProgramViewBase::ManipulatorProgramViewBase()
{
    impl = new Impl(this);

    registerBaseStatementDelegates();
}


ManipulatorProgramViewBase::Impl::Impl(ManipulatorProgramViewBase* self)
    : self(self),
      targetItemPicker(self)
{
    setupWidgets();

    statementItemOperationCallCounter = 0;

    targetItemPicker.sigTargetItemChanged().connect(
        [&](ManipulatorProgramItemBase* item){ setProgramItem(item); });

    defaultStatementDelegate = new StatementDelegate;
}


ManipulatorProgramViewBase::~ManipulatorProgramViewBase()
{
    delete impl;
}


ManipulatorProgramViewBase::Impl::~Impl()
{

}


void ManipulatorProgramViewBase::Impl::setupWidgets()
{
    self->setDefaultLayoutArea(View::LEFT_TOP);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);

    auto hbox = new QHBoxLayout;
    hbox->addLayout(&buttonBox[0]);
    hbox->addStretch();
    optionMenuButton.setText(_("*"));
    optionMenuButton.sigClicked().connect([&](){ onOptionMenuClicked(); });
    hbox->addWidget(&optionMenuButton);
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addLayout(&buttonBox[1]);
    hbox->addStretch();
    vbox->addLayout(hbox);

    programNameLabel.setFrameStyle(QFrame::Box | QFrame::Sunken);
    vbox->addWidget(&programNameLabel);

    // TreeWidget setup
    setColumnCount(NumColumns);
    setFrameShape(QFrame::NoFrame);
    setHeaderHidden(true);
    setRootIsDecorated(true);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setSelectionMode(QAbstractItemView::ExtendedSelection);
    setDragDropMode(QAbstractItemView::InternalMove);
    setDropIndicatorShown(true);
    setTabKeyNavigation(true);
    setAllColumnsShowFocus(true);
    //setAlternatingRowColors(true);
    setEditTriggers(
        QAbstractItemView::DoubleClicked |
        QAbstractItemView::SelectedClicked |
        QAbstractItemView::EditKeyPressed |
        QAbstractItemView::AnyKeyPressed);
    //setStyle(new TreeWidgetStyle(this));

    mainDelegate = new ProgramViewDelegate(this);

    QItemEditorFactory* factory = new QItemEditorFactory;
    QItemEditorCreatorBase* stringListEditorCreator =
        new QStandardItemEditorCreator<StringListComboBox>();
    factory->registerEditor(QVariant::StringList, stringListEditorCreator);
    mainDelegate->setItemEditorFactory(factory);
    
    setItemDelegate(mainDelegate);

    auto& rheader = *header();
    rheader.setMinimumSectionSize(0);
    rheader.setStretchLastSection(false);
    rheader.setSectionResizeMode(0, QHeaderView::ResizeToContents);
    rheader.setSectionResizeMode(1, QHeaderView::ResizeToContents);
    rheader.setSectionResizeMode(2, QHeaderView::Stretch);
    
    sigCurrentItemChanged().connect(
        [&](QTreeWidgetItem* current, QTreeWidgetItem* previous){
            onCurrentTreeWidgetItemChanged(current, previous); });

    sigItemClicked().connect(
        [&](QTreeWidgetItem* item, int column){
            onTreeWidgetItemClicked(item, column); });

    sigRowsAboutToBeRemoved().connect(
        [&](const QModelIndex& parent, int start, int end){
            onRowsAboutToBeRemoved(parent, start, end); });

    sigRowsRemoved().connect(
        [&](const QModelIndex& parent, int start, int end){
            onRowsRemoved(parent, start, end); });
    
    sigRowsInserted().connect(
        [&](const QModelIndex& parent, int start, int end){
            onRowsInserted(parent, start, end);  });
    
    vbox->addWidget(this);
    
    self->setLayout(vbox);

    optionMenuManager.setNewPopupMenu(this);
    optionMenuManager.addItem(_("Refresh"))->sigTriggered().connect(
        [&](){ updateStatementTree(); });
}


void ManipulatorProgramViewBase::registerStatementDelegate
(type_index statementType, StatementDelegate* delegate)
{
    delegate->impl->setView(impl);
    impl->statementDelegateMap[statementType] = delegate;
}


ManipulatorProgramViewBase::StatementDelegate*
ManipulatorProgramViewBase::Impl::findStatementDelegate(ManipulatorStatement* statement)
{
    auto iter = statementDelegateMap.find(typeid(*statement));
    if(iter != statementDelegateMap.end()){
        return iter->second;
    }
    return defaultStatementDelegate;
}


void ManipulatorProgramViewBase::addStatementButton(QWidget* button, int row)
{
    impl->buttonBox[row].addWidget(button);
}


void ManipulatorProgramViewBase::onDeactivated()
{
    impl->currentStatement = nullptr;
    impl->prevCurrentStatement = nullptr;
}


void ManipulatorProgramViewBase::Impl::onOptionMenuClicked()
{
    optionMenuManager.popupMenu()->popup(optionMenuButton.mapToGlobal(QPoint(0,0)));
}


ScopedCounter ManipulatorProgramViewBase::Impl::scopedCounterOfStatementItemOperationCall()
{
    return ScopedCounter(statementItemOperationCallCounter);
}


bool ManipulatorProgramViewBase::Impl::isDoingStatementItemOperation() const
{
    return (statementItemOperationCallCounter > 0);
}


void ManipulatorProgramViewBase::Impl::setProgramItem(ManipulatorProgramItemBase* item)
{
    programConnections.disconnect();
    programItem = item;
    currentStatement = nullptr;

    bool accepted = self->onCurrentProgramItemChanged(item);
    if(!accepted){
        programItem = nullptr;
    }

    if(!programItem){
        programNameLabel.setStyleSheet("");
        programNameLabel.setText("---");

    } else {
        auto program = programItem->program();
        
        programConnections.add(
            programItem->sigNameChanged().connect(
                [&](const std::string&){ setProgramItem(programItem); }));

        programConnections.add(
            program->sigStatementInserted().connect(
                [&](ManipulatorProgram* program, ManipulatorProgram::iterator iter){
                    onStatementInserted(program, iter); }));

        programConnections.add(
            program->sigStatementRemoved().connect(
                [&](ManipulatorProgram* program, ManipulatorStatement* statement){
                    onStatementRemoved(program, statement); }));
        
        programNameLabel.setStyleSheet("font-weight: bold");
        programNameLabel.setText(programItem->name().c_str());

    }

    updateStatementTree();
}


void ManipulatorProgramViewBase::Impl::updateStatementTree()
{
    auto counter = scopedCounterOfStatementItemOperationCall();
    
    clear();
    statementItemMap.clear();
    
    if(programItem){
        addProgramStatementsToTree(programItem->program(), invisibleRootItem());
    }
}


void ManipulatorProgramViewBase::Impl::addProgramStatementsToTree
(ManipulatorProgram* program, QTreeWidgetItem* parentItem)
{
    auto counter = scopedCounterOfStatementItemOperationCall();
    parentItem->setExpanded(true);
    for(auto& statement : *program){
        auto statementItem = new StatementItem(statement, this);
        parentItem->addChild(statementItem);
        if(auto structured = dynamic_cast<StructuredStatement*>(statement.get())){
            if(auto lowerLevelProgram = structured->lowerLevelProgram()){
                addProgramStatementsToTree(lowerLevelProgram, statementItem);
            }
        }
    }
}


ManipulatorProgramItemBase* ManipulatorProgramViewBase::currentProgramItem()
{
    return impl->programItem;
}


ManipulatorStatement* ManipulatorProgramViewBase::currentStatement()
{
    return impl->currentStatement;
}


SignalProxy<void(ManipulatorStatement* statement)> ManipulatorProgramViewBase::sigCurrentStatementChanged()
{
    return impl->sigCurrentStatementChanged;
}


void ManipulatorProgramViewBase::Impl::onCurrentTreeWidgetItemChanged
(QTreeWidgetItem* current, QTreeWidgetItem* previous)
{
    if(auto statementItem = dynamic_cast<StatementItem*>(previous)){
        prevCurrentStatement = statementItem->statement();
    }
    if(auto statementItem = dynamic_cast<StatementItem*>(current)){
        setCurrentStatement(statementItem->statement());
    }
}


void ManipulatorProgramViewBase::Impl::setCurrentStatement(ManipulatorStatement* statement)
{
    currentStatement = statement;
    self->onCurrentStatementChanged(statement);
    sigCurrentStatementChanged(statement);
    self->onCurrentStatementActivated(statement);
}


void ManipulatorProgramViewBase::onCurrentStatementChanged(ManipulatorStatement*)
{

}


void ManipulatorProgramViewBase::Impl::onTreeWidgetItemClicked(QTreeWidgetItem* item, int /* column */)
{
    if(auto statementItem = dynamic_cast<StatementItem*>(item)){
        auto statement = statementItem->statement();
        // If the clicked statement is different from the current one,
        // onCurrentTreeWidgetItemChanged is processed
        if(statement == prevCurrentStatement){
            self->onCurrentStatementActivated(statement);
        }
    }
}


void ManipulatorProgramViewBase::onCurrentStatementActivated(ManipulatorStatement*)
{

}


StatementItem* ManipulatorProgramViewBase::Impl::statementItemFromStatement(ManipulatorStatement* statement)
{
    auto iter = statementItemMap.find(statement);
    if(iter != statementItemMap.end()){
        return iter->second;
    }
    return nullptr;
}


bool ManipulatorProgramViewBase::insertStatement(ManipulatorStatement* statement, int insertionType)
{
    return impl->insertStatement(statement, insertionType);
}


bool ManipulatorProgramViewBase::Impl::insertStatement(ManipulatorStatement* statement, int insertionType)
{
    if(programItem){
        auto program = programItem->program();
        ManipulatorProgram::iterator pos;
        StructuredStatement* parentStatement = nullptr;
        auto selected = selectedItems();
        if(selected.empty()){
            if(insertionType == BeforeTargetPosition){
                pos = program->begin();
            } else {
                pos = program->end();
            }
        } else {
            auto lastSelectedItem = static_cast<StatementItem*>(selected.back());
            parentStatement = lastSelectedItem->getParentStatement();
            if(parentStatement){
                program = parentStatement->lowerLevelProgram();
            }
            auto index = indexFromItem(lastSelectedItem);
            pos = program->begin() + index.row();

            auto dummyStatementAtInsertionPosition = lastSelectedItem->statement<DummyStatement>();
            if(dummyStatementAtInsertionPosition){
                pos = program->remove(pos);
                //programItem->sigStatementRemoved()(dummyStatementAtInsertionPosition, parentStatement);
            }
            if(insertionType == AfterTargetPosition && !dummyStatementAtInsertionPosition){
                ++pos;
            }
        }
        auto inserted = program->insert(pos, statement);
        //programItem->sigStatementAdded()(inserted, parentStatement);
        //programItem->suggestFileUpdate();
        return true;
    }
    return false;
}


void ManipulatorProgramViewBase::Impl::onStatementInserted
(ManipulatorProgram* program, ManipulatorProgram::iterator iter)
{
    auto counter = scopedCounterOfStatementItemOperationCall();

    QTreeWidgetItem* parentItem;
    if(auto holderStatement = program->holderStatement()){
        parentItem = statementItemFromStatement(holderStatement);
    } else {
        parentItem = invisibleRootItem();
    }

    auto statement = *iter;
    auto statementItem = new StatementItem(statement, this);

    bool added = false;
    auto nextIter = ++iter;
    if(nextIter == program->end()){
        parentItem->addChild(statementItem);
        added = true;
    } else {
        if(auto nextItem = statementItemFromStatement(*nextIter)){
            parentItem->insertChild(parentItem->indexOfChild(nextItem), statementItem);
            added = true;
        }
    }

    if(added){
        if(auto structured = dynamic_cast<StructuredStatement*>(statement.get())){
            if(auto lowerLevelProgram = structured->lowerLevelProgram()){
                addProgramStatementsToTree(lowerLevelProgram, statementItem);
            }
        }
    }
}


void ManipulatorProgramViewBase::Impl::onStatementRemoved
(ManipulatorProgram* program, ManipulatorStatement* statement)
{
    auto counter = scopedCounterOfStatementItemOperationCall();
    auto iter = statementItemMap.find(statement);
    if(iter != statementItemMap.end()){
        auto statementItem = iter->second;
        QTreeWidgetItem* parentItem;
        if(auto holderStatement = program->holderStatement()){
            parentItem = statementItemFromStatement(holderStatement);
        } else {
            parentItem = invisibleRootItem();
        }
        parentItem->takeChild(parentItem->indexOfChild(statementItem));
        delete statementItem;
    }
}


void ManipulatorProgramViewBase::Impl::forEachStatementInTreeEditEvent
(const QModelIndex& parent, int start, int end,
 function<void(StructuredStatement* parentStatement, ManipulatorProgram* program,
               int index, ManipulatorStatement* statement)> func)
{
    if(!programItem || isDoingStatementItemOperation()){
        return;
    }
    QTreeWidgetItem* parentItem = nullptr;
    StructuredStatement* parentStatement = nullptr;
    ManipulatorProgram* program = nullptr;
    
    if(!parent.isValid()){
        parentItem = invisibleRootItem();
        program = programItem->program();
    } else {
        parentItem = itemFromIndex(parent);
        auto parentStatementItem = static_cast<StatementItem*>(parentItem);
        parentStatement = parentStatementItem->statement<StructuredStatement>();
        if(parentStatement){
            program = parentStatement->lowerLevelProgram();
        }
    }
    if(program){
        for(int i = start; i <= end; ++i){
            auto item = static_cast<StatementItem*>(parentItem->child(i));
            func(parentStatement, program, i, item->statement());
        }
        programItem->suggestFileUpdate();
    }
}
    

void ManipulatorProgramViewBase::Impl::onRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end)
{
    forEachStatementInTreeEditEvent(
        parent, start, end,
        [&](StructuredStatement*, ManipulatorProgram* program, int, ManipulatorStatement* statement){
            programConnections.block();
            program->remove(statement);
            programConnections.unblock();
        });
}


void ManipulatorProgramViewBase::Impl::onRowsRemoved(const QModelIndex& parent, int start, int end)
{
    if(isDoingStatementItemOperation()){
        return;
    }
    // Keep at least one dummy statement in a control structure statement
    if(parent.isValid()){
        if(auto structured = itemFromIndex(parent)->statement<StructuredStatement>()){
            auto program = structured->lowerLevelProgram();
            if(program->empty()){
                program->append(new DummyStatement);
                //programItem->sigStatementAdded()(inserted, structured);
            }
        }
    }
}


void ManipulatorProgramViewBase::Impl::onRowsInserted(const QModelIndex& parent, int start, int end)
{
    forEachStatementInTreeEditEvent(
        parent, start, end,
        [&](StructuredStatement* parentStatement, ManipulatorProgram* program, int index,
            ManipulatorStatement* statement){

            programConnections.block();
            program->insert(program->begin() + index, statement);
            programConnections.unblock();

            // The dummy statement at the insertion position is overwritten
            if(index == end && parentStatement){
                if(!removeDummyStatementAroundInsertionPosition(parentStatement, program, start, -1)){
                    removeDummyStatementAroundInsertionPosition(parentStatement, program, end, +1);
                }
            }
        });
}


bool ManipulatorProgramViewBase::Impl::removeDummyStatementAroundInsertionPosition
(StructuredStatement* parentStatement, ManipulatorProgram* program, int index, int direction)
{
    auto pos = program->begin() + index;
    if(direction < 0 && pos == program->begin() || direction > 0 && pos == program->end()){
        return false;
    }
    pos += direction;
    auto statement = *pos;
    if(auto dummy = dynamic_cast<DummyStatement*>(statement.get())){
        program->remove(pos);
        //programItem->sigStatementRemoved()(dummy, parentStatement);
        return true;
    }
    return false;
}


void ManipulatorProgramViewBase::Impl::keyPressEvent(QKeyEvent* event)
{
    bool processed = true;

    switch(event->key()){
    case Qt::Key_Escape:
        clearSelection();
        break;
    case Qt::Key_Delete:
        copySelectedStatements(true);
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
        case Qt::Key_X:
            copySelectedStatements(true);
            break;
        case Qt::Key_C:
            copySelectedStatements(false);
            break;
        case Qt::Key_V:
            pasteStatements();
            break;
        }
    }

    if(!processed){
        TreeWidget::keyPressEvent(event);
    }
}            


void ManipulatorProgramViewBase::Impl::mousePressEvent(QMouseEvent* event)
{
    TreeWidget::mousePressEvent(event);

    if(event->button() == Qt::RightButton){
        showContextMenu(event->globalPos());
    }
}


void ManipulatorProgramViewBase::Impl::showContextMenu(QPoint globalPos)
{
    contextMenuManager.setNewPopupMenu(this);

    contextMenuManager.addItem(_("Insert empty line"))
        ->sigTriggered().connect([=](){ insertStatement(new DummyStatement, BeforeTargetPosition); });

    contextMenuManager.addSeparator();

    contextMenuManager.addItem(_("Cut"))
        ->sigTriggered().connect([=](){ copySelectedStatements(true); });

    contextMenuManager.addItem(_("Copy"))
        ->sigTriggered().connect([=](){ copySelectedStatements(false); });

    auto pasteAction = contextMenuManager.addItem(_("Paste"));
    if(statementsToPaste.empty()){
        pasteAction->setEnabled(false);
    } else {
        pasteAction->sigTriggered().connect([=](){ pasteStatements(); });
    }
    
    contextMenuManager.popupMenu()->popup(globalPos);
}


void ManipulatorProgramViewBase::Impl::copySelectedStatements(bool doCut)
{
    vector<StatementItem*> selectedStatementTops;
    auto selectedItems_ = selectedItems();
    for(auto& item : selectedItems_){
        auto statementItem = static_cast<StatementItem*>(item);
        if(statementItem->statement<DummyStatement>()){
            continue;
        }
        bool isTop = true;
        auto parent = item->parent();
        while(parent){
            if(selectedItems_.contains(parent)){
                isTop = false;
                break;
            }
            parent = parent->parent();
        }
        if(isTop){
            selectedStatementTops.push_back(statementItem);
        }
    }

    if(!selectedStatementTops.empty()){
        auto counter = scopedCounterOfStatementItemOperationCall();
        statementsToPaste.clear();
        for(auto& statementItem : selectedStatementTops){
            auto statement = statementItem->statement();
            statementsToPaste.push_back(statement->clone());

            if(doCut){
                ManipulatorProgram* program;
                auto parentStatement = statementItem->getParentStatement();
                if(parentStatement){
                    program = parentStatement->lowerLevelProgram();
                } else {
                    program = programItem->program();
                }
                program->remove(statement);
                //programItem->sigStatementRemoved()(statement, parentStatement);
            }
        }
        programItem->suggestFileUpdate();
    }
}


void ManipulatorProgramViewBase::Impl::pasteStatements()
{
    auto counter = scopedCounterOfStatementItemOperationCall();

    ManipulatorProgram* program = programItem->program();
    StatementItem* pastePositionItem = nullptr;
    StructuredStatement* parentStatement = nullptr;
    ManipulatorProgram::iterator pos;
    
    auto selected = selectedItems();
    if(!selected.empty()){
        pastePositionItem = static_cast<StatementItem*>(selected.back());
    }
    if(!pastePositionItem){
        pos = program->end();
    } else {
        parentStatement = pastePositionItem->getParentStatement();
        if(parentStatement){
            program = parentStatement->lowerLevelProgram();
        }
        auto index = indexFromItem(pastePositionItem);
        int row = index.row();
        if(auto dummyStatement = pastePositionItem->statement<DummyStatement>()){
            pos = program->remove(program->begin() + row);
            //programItem->sigStatementRemoved()(dummyStatement, parentStatement);
        } else {
            pos = program->begin() + row + 1;
        }
    }
    
    for(auto& statement : statementsToPaste){
        pos = program->insert(pos, statement->clone());
        //programItem->sigStatementAdded()(pos, parentStatement);
        ++pos;
    }
    
    programItem->suggestFileUpdate();
}


bool ManipulatorProgramViewBase::storeState(Archive& archive)
{
    impl->targetItemPicker.storeTargetItem(archive, "currentProgram");
    return true;
}


bool ManipulatorProgramViewBase::restoreState(const Archive& archive)
{
    impl->targetItemPicker.restoreTargetItemLater(archive, "currentProgram");
    return true;
}
