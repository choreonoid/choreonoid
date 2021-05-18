#include "MprProgramViewBase.h"
#include "MprProgramItemBase.h"
#include "MprProgram.h"
#include "MprBasicStatements.h"
#include "MprPositionStatement.h"
#include "MprControllerItemBase.h"
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/TargetItemPicker>
#include <cnoid/TreeWidget>
#include <cnoid/MessageView>
#include <cnoid/TimeBar>
#include <cnoid/BodyItem>
#include <cnoid/ControllerLogItem>
#include <cnoid/StringListComboBox>
#include <cnoid/Buttons>
#include <cnoid/ConnectionSet>
#include <cnoid/Archive>
#include <QBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QProxyStyle>
#include <QStyledItemDelegate>
#include <QStyle>
#include <QPainter>
#include <QItemEditorFactory>
#include <QStandardItemEditorCreator>
#include <fmt/format.h>
#include <unordered_map>
#include <array>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

static constexpr int NumColumns = 4;

class StatementItem : public QTreeWidgetItem
{
    MprStatementPtr statement_;
public:
    MprProgramViewBase::Impl* viewImpl;
    MprProgramViewBase::StatementDelegate* delegate;

    StatementItem(MprStatement* statement, MprProgram* program, MprProgramViewBase::Impl* viewImpl);
    ~StatementItem();
    virtual QVariant data(int column, int role) const override;
    virtual void setData(int column, int role, const QVariant& value) override;
    StatementItem* getParentStatementItem() const { return static_cast<StatementItem*>(parent()); }
    MprStructuredStatement* getParentStatement() const;
    MprProgram* getProgram() const;
    template<class StatementType>
    ref_ptr<StatementType> statement() const { return dynamic_pointer_cast<StatementType>(statement_); }
    MprStatementPtr statement() const { return statement_; }
};


class ProgramViewDelegate : public QStyledItemDelegate
{
    MprProgramViewBase::Impl* viewImpl;
public:
    ProgramViewDelegate(MprProgramViewBase::Impl* viewImpl);
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
    MprProgramViewBase::Impl* viewImpl;
public:
    TreeWidgetStyle(MprProgramViewBase::Impl* viewImpl);
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

class MprProgramViewBase::StatementDelegate::Impl
{
public:
    MprProgramViewBase::StatementDelegate* self;
    ProgramViewDelegate* mainDelegate;
    const StatementItem* currentItem;
    QWidget* currentParentWidget;
    const QStyleOptionViewItem* pCurrentOption;
    QAbstractItemModel* currentModel;
    const QModelIndex* pCurrentModelIndex;

    Impl(MprProgramViewBase::StatementDelegate* self);
    void setView(MprProgramViewBase::Impl* viewImpl);
    int actualLabelSpan(MprStatement* statement, int column) const;
};

class MprProgramViewBase::Impl : public TreeWidget
{
public:
    MprProgramViewBase* self;
    TargetItemPicker<MprProgramItemBase> targetItemPicker;
    MprProgramItemBasePtr currentProgramItem;
    shared_ptr<const string> logTopLevelProgramName;
    weak_ptr<ReferencedObjectSeq> invalidLogSeq;
    unordered_map<MprStatementPtr, StatementItem*> statementItemMap;
    MprDummyStatementPtr dummyStatement;
    int statementItemOperationCallCounter;
    bool isOnTimeChanged;
    ScopedConnection timeBarConnection;
    ScopedConnectionSet programConnections;

    std::vector<MprStatementPtr> selectedStatements;
    Signal<void(std::vector<MprStatementPtr>& statements)> sigSelectedStatementsChanged;

    // The following "current statement" is not necessarily same as the current item of the tree widget.
    // For example, when multiple statements are selected and one of them is unselected with ctrl + left click,
    // the unselected item will be the current item of the tree widget, which is not an intuitive behavior.
    // The following current statement corresponds to the last and still selected item in this case.
    // It is always one of the selected items.
    MprStatementPtr currentStatement;
    Signal<void(MprStatement* statement)> sigCurrentStatementChanged;

    // The following "active statement" corresponds to the current item of the tree widget.
    // The last clicked item becomes the active statement.
    MprStatementPtr prevActiveStatement;
    
    vector<MprStatementPtr> statementsToPaste;
    weak_ref_ptr<MprStatement> weak_errorStatement;
    BodySyncMode bodySyncMode;

    bool isJustAfterDoubleClicked;

    /*
    struct ExpansionState {
        ScopedConnection statementConnection;
        bool expanded;
    };
    */
    typedef map<MprStructuredStatementPtr, bool> ExpansionStateMap;
    struct ProgramState {
        ScopedConnectionSet programConnections;
        ExpansionStateMap expansionStateMap;
    };
    map<MprProgramItemBasePtr, ProgramState> programStateMap;
    
    QLabel programNameLabel;
    QHBoxLayout buttonBox[3];
    QSize buttonIconSize;
    ProgramViewDelegate* mainDelegate;
    ref_ptr<StatementDelegate> defaultStatementDelegate;
    unordered_map<type_index, ref_ptr<StatementDelegate>> statementDelegateMap;

    MenuManager contextMenuManager;
    PolymorphicMprStatementFunctionSet contextMenuFunctions;

    Impl(MprProgramViewBase* self);
    ~Impl();
    void setupWidgets();
    StatementDelegate* findStatementDelegate(MprStatement* statement);
    ScopedCounter scopedCounterOfStatementItemOperationCall();
    bool isDoingStatementItemOperation() const;
    void setProgramItem(MprProgramItemBase* item);
    void storeExpansionStates(ExpansionStateMap& expansionStateMap);
    void storeExpansionStatesIter(MprProgram* program, ExpansionStateMap& expansionStateMap);
    void clearStatementTree();
    void addStatementsToTree(
        MprProgram* program, QTreeWidgetItem* parentItem, MprStructuredStatement* parentStatement,
        ExpansionStateMap* expansionStateMap);
    void setCurrentStatement(MprStatement* statement);
    void setErrorStatement(MprStatement* statement);
    void onItemSelectionChanged();
    void onCurrentTreeWidgetItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous);
    void onTreeWidgetItemClicked(QTreeWidgetItem* item, int /* column */);
    void onTreeWidgetItemDoubleClicked(QTreeWidgetItem* item, int /* column */);
    MprStatement* findStatementAtHierachicalPosition(const vector<int>& position);
    MprStatement* findStatementAtHierachicalPositionIter(
        const vector<int>& position, MprProgram* program, int level);
    bool findControllerItemAndLogItem(
        MprControllerItemBase*& controllerItem, ControllerLogItem*& logItem);
    shared_ptr<ReferencedObjectSeq> findLogSeq();
    bool onTimeChanged(double time);
    bool seekToLogPosition(MprControllerItemBase* controllerItem, MprControllerLog* log);
    StatementItem* findStatementItem(MprStatement* statement);
    bool insertStatement(MprStatement* statement, int insertionType);
    void onStatementInserted(MprProgram::iterator iter);
    void onStatementRemoved(MprProgram* program, MprStatement* statement);
    void onStatementUpdated(MprStatement* statement);
    void forEachStatementInTreeEditEvent(
        const QModelIndex& parent, int start, int end,
        function<void(MprStructuredStatement* parentStatement, MprProgram* program,
                      int index, MprStatement* statement)> func);
    void onRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end);
    void onRowsRemoved(const QModelIndex& parent, int start, int end);
    void onRowsInserted(const QModelIndex& parent, int start, int end);
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    void showContextMenu(MprStatement* statement, QPoint globalPos);
    void setBaseContextMenu(MenuManager& menuManager);
    void copySelectedStatements(bool doCut);
    void pasteStatements();

    QModelIndex indexFromItem(QTreeWidgetItem* item, int column = 0) const {
        return TreeWidget::indexFromItem(item, column);
    }
    StatementItem* itemFromIndex(const QModelIndex& index) const {
        if(index.isValid()){
            return static_cast<StatementItem*>(TreeWidget::itemFromIndex(index));
        }
        return nullptr;
    }
};

}


namespace {

StatementItem::StatementItem(MprStatement* statement_, MprProgram* program, MprProgramViewBase::Impl* viewImpl)
    : statement_(statement_),
      viewImpl(viewImpl)
{
    if(statement_ != viewImpl->dummyStatement){
        viewImpl->statementItemMap[statement_] = this;
    }
    delegate = viewImpl->findStatementDelegate(statement_);
    delegate->activateStatement(statement_);
    
    Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsSelectable;

    if(program && program->isEditingEnabled()){
        flags |= Qt::ItemIsEditable | Qt::ItemIsDragEnabled;
    }
    if(statement<MprStructuredStatement>()){
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
            QString label(statement()->label(column).c_str());
            if(label.size() < 6){
                label.append(QString(6 - label.size(), ' '));
            }
            return label;
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


MprStructuredStatement* StatementItem::getParentStatement() const
{
    if(auto parentStatementItem = getParentStatementItem()){
        return parentStatementItem->statement<MprStructuredStatement>();
    }
    return nullptr;
}


MprProgram* StatementItem::getProgram() const
{
    if(auto parentStatement = getParentStatement()){
        return parentStatement->lowerLevelProgram();
    }
    if(viewImpl->currentProgramItem){
        return viewImpl->currentProgramItem->program();
    }
    return nullptr;
}


ProgramViewDelegate::ProgramViewDelegate(MprProgramViewBase::Impl* viewImpl)
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
    return delegate->createEditor(statement, column, parent);
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
        //rect.setWidth(rect.width() * 1.25);

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


TreeWidgetStyle::TreeWidgetStyle(MprProgramViewBase::Impl* viewImpl)
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

}


MprProgramViewBase::StatementDelegate::StatementDelegate()
{
    impl = new Impl(this);
}


MprProgramViewBase::StatementDelegate::Impl::Impl
(MprProgramViewBase::StatementDelegate* self)
    : self(self)
{
    mainDelegate = nullptr;
    currentItem = nullptr;
    currentParentWidget = nullptr;
    pCurrentOption = nullptr;
    currentModel = nullptr;
    pCurrentModelIndex = nullptr;
}


MprProgramViewBase::StatementDelegate::~StatementDelegate()
{
    delete impl;
}


void MprProgramViewBase::StatementDelegate::Impl::setView
(MprProgramViewBase::Impl* viewImpl)
{
    mainDelegate = viewImpl->mainDelegate;
}


int MprProgramViewBase::StatementDelegate::Impl::actualLabelSpan
(MprStatement* statement, int column) const
{
    int span = self->labelSpan(statement, column);
    if(span == SpanToLast){
        span = (NumColumns - column);
    }
    return span;
}


void MprProgramViewBase::StatementDelegate::activateStatement(MprStatement* /* statement */) const
{

}


int MprProgramViewBase::StatementDelegate::labelSpan
(MprStatement* /* statement */, int /* column */) const
{
    return 1;
}


QVariant MprProgramViewBase::StatementDelegate::dataOfEditRole
(MprStatement* statement, int column) const
{
    int span = labelSpan(statement, column);
    if(span == 1){
        return QString(statement->label(column).c_str());
    }
    return QVariant();
}


void MprProgramViewBase::StatementDelegate::setDataOfEditRole
(MprStatement* /* statement */, int /* column */, const QVariant& /* value */) const
{

}


QWidget* MprProgramViewBase::StatementDelegate::createEditor
(MprStatement* /* statement */, int /* column */, QWidget* /* parent */) const
{
    return nullptr;
}


QWidget* MprProgramViewBase::StatementDelegate::createDefaultEditor() const
{
    return impl->mainDelegate->QStyledItemDelegate::createEditor(
        impl->currentParentWidget, *impl->pCurrentOption, *impl->pCurrentModelIndex);
}


void MprProgramViewBase::StatementDelegate::setEditorData
(MprStatement* /* statement */, int /* column */, QWidget* editor) const
{
    impl->mainDelegate->QStyledItemDelegate::setEditorData(editor, *impl->pCurrentModelIndex);
}


void MprProgramViewBase::StatementDelegate::setStatementData
(MprStatement* /* statement */, int /* column */, QWidget* editor ) const
{
    impl->mainDelegate->QStyledItemDelegate::setModelData(editor, impl->currentModel, *impl->pCurrentModelIndex);
}


MprProgramViewBase::MprProgramViewBase()
{
    impl = new Impl(this);

    registerBaseStatementDelegates();

    customizeContextMenu<MprStatement>(
        [this](MprStatement*, MenuManager& menuManager, MprStatementFunctionDispatcher){
            impl->setBaseContextMenu(menuManager); });
}


MprProgramViewBase::Impl::Impl(MprProgramViewBase* self)
    : self(self),
      targetItemPicker(self)
{
    setupWidgets();

    targetItemPicker.sigTargetItemChanged().connect(
        [&](MprProgramItemBase* item){ setProgramItem(item); });

    dummyStatement = new MprDummyStatement;
    statementItemOperationCallCounter = 0;
    isOnTimeChanged = false;
    bodySyncMode = DirectBodySync;
    isJustAfterDoubleClicked = false;
    defaultStatementDelegate = new StatementDelegate;
}


MprProgramViewBase::~MprProgramViewBase()
{
    delete impl;
}


MprProgramViewBase::Impl::~Impl()
{

}


void MprProgramViewBase::Impl::setupWidgets()
{
    self->setDefaultLayoutArea(View::LEFT_TOP);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);

    QHBoxLayout* hbox;
    hbox = new QHBoxLayout;
    hbox->addLayout(&buttonBox[0]);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addLayout(&buttonBox[1]);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addLayout(&buttonBox[2]);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    programNameLabel.setFrameStyle(QFrame::Box | QFrame::Sunken);
    vbox->addWidget(&programNameLabel);

    // TreeWidget setup
    setColumnCount(NumColumns);
    setFrameShape(QFrame::NoFrame);
    setHeaderHidden(true);
    setRootIsDecorated(true);
    setAutoScroll(false);
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
    rheader.setSectionResizeMode(2, QHeaderView::ResizeToContents);
    rheader.setSectionResizeMode(3, QHeaderView::Stretch);
    sigSectionResized().connect([&](int, int, int){ updateGeometry(); });

    sigItemSelectionChanged().connect([&](){ onItemSelectionChanged(); });

    sigCurrentItemChanged().connect(
        [&](QTreeWidgetItem* current, QTreeWidgetItem* previous){
            onCurrentTreeWidgetItemChanged(current, previous); });

    sigItemClicked().connect(
        [&](QTreeWidgetItem* item, int column){
            onTreeWidgetItemClicked(item, column); });

    sigItemDoubleClicked().connect(
        [&](QTreeWidgetItem* item, int column){
            onTreeWidgetItemDoubleClicked(item, column); });

    sigRowsInserted().connect(
        [&](const QModelIndex& parent, int start, int end){
            onRowsInserted(parent, start, end);  });

    sigRowsAboutToBeRemoved().connect(
        [&](const QModelIndex& parent, int start, int end){
            onRowsAboutToBeRemoved(parent, start, end); });

    sigRowsRemoved().connect(
        [&](const QModelIndex& parent, int start, int end){
            onRowsRemoved(parent, start, end); });
    
    vbox->addWidget(this);
    
    self->setLayout(vbox);

    int s = self->style()->pixelMetric(QStyle::PM_ButtonIconSize);
    buttonIconSize = QSize(s, s);
    if(s != 24){
        if(s >= 16 && s < 24){
            buttonIconSize = QSize(24, 24);
        }
    }
}


void MprProgramViewBase::onActivated()
{
    // TODO: Use TimeSyncItemEngine instead of using the time bar directly
    auto timeBar = TimeBar::instance();
    impl->timeBarConnection =
        timeBar->sigTimeChanged().connect(
            [this](double time){ return impl->onTimeChanged(time); });
    impl->onTimeChanged(timeBar->time());
}


void MprProgramViewBase::onDeactivated()
{
    impl->timeBarConnection.disconnect();
    impl->currentStatement.reset();
    impl->prevActiveStatement.reset();
}


void MprProgramViewBase::registerStatementDelegate
(type_index statementType, StatementDelegate* delegate)
{
    delegate->impl->setView(impl);
    impl->statementDelegateMap[statementType] = delegate;
}


MprProgramViewBase::StatementDelegate*
MprProgramViewBase::Impl::findStatementDelegate(MprStatement* statement)
{
    auto iter = statementDelegateMap.find(typeid(*statement));
    if(iter != statementDelegateMap.end()){
        return iter->second;
    }
    return defaultStatementDelegate;
}


void MprProgramViewBase::setBodySyncMode(BodySyncMode mode)
{
    if(mode != impl->bodySyncMode){
        impl->bodySyncMode = mode;
        if(!mode && impl->currentProgramItem){
            impl->currentProgramItem->clearSuperimposition();
        }
    }
}


MprProgramViewBase::BodySyncMode MprProgramViewBase::bodySyncMode() const
{
    return impl->bodySyncMode;
}


void MprProgramViewBase::addEditButton(ToolButton* button, int row)
{
    button->setIconSize(impl->buttonIconSize);
    impl->buttonBox[row].addWidget(button);
}


void MprProgramViewBase::onAttachedMenuRequest(MenuManager& menuManager)
{
    menuManager.addItem(_("Refresh"))->sigTriggered().connect(
        [&](){ updateStatementTree(); });

    menuManager.addItem(_("Renumbering"))->sigTriggered().connect(
        [&](){
            if(impl->currentProgramItem){
                impl->currentProgramItem->program()->renumberPositionIds();
                impl->currentProgramItem->notifyUpdate();
                impl->currentProgramItem->suggestFileUpdate();
                updateStatementTree();
            }
        });
}
    

ScopedCounter MprProgramViewBase::Impl::scopedCounterOfStatementItemOperationCall()
{
    return ScopedCounter(statementItemOperationCallCounter);
}


bool MprProgramViewBase::Impl::isDoingStatementItemOperation() const
{
    return (statementItemOperationCallCounter > 0);
}


bool MprProgramViewBase::checkCurrentProgramItem() const
{
    if(!impl->currentProgramItem){
        showWarningDialog(_("Program item is not specified."));
        return false;
    }
    return true;
}


void MprProgramViewBase::Impl::setProgramItem(MprProgramItemBase* item)
{
    if(item == currentProgramItem){
        return;
    }
    
    programConnections.disconnect();
    if(currentProgramItem){
        currentProgramItem->clearSuperimposition();
    }

    if(currentProgramItem){
        storeExpansionStates(programStateMap[currentProgramItem].expansionStateMap);
    }
    
    currentProgramItem = item;
    logTopLevelProgramName.reset();
    currentStatement.reset();
    prevActiveStatement.reset();
    weak_errorStatement.reset();

    bool accepted = self->onCurrentProgramItemChanged(item);
    if(!accepted){
        currentProgramItem = nullptr;
    }

    clearStatementTree();

    if(!currentProgramItem){
        programNameLabel.setStyleSheet("");
        programNameLabel.setText("---");
        return;
    }

    auto program = currentProgramItem->program();
        
    programConnections.add(
        currentProgramItem->sigNameChanged().connect(
            [&](const std::string&){ setProgramItem(currentProgramItem); }));
    
    programConnections.add(
        program->sigStatementInserted().connect(
            [&](MprProgram::iterator iter){
                onStatementInserted(iter); }));
    
    programConnections.add(
        program->sigStatementRemoved().connect(
            [&](MprStatement* statement, MprProgram* program){
                onStatementRemoved(program, statement); }));
    
    programConnections.add(
        program->sigStatementUpdated().connect(
            [&](MprStatement* statement){
                onStatementUpdated(statement); }));
    
    programNameLabel.setStyleSheet("font-weight: bold");

    if(auto bodyItem = currentProgramItem->targetBodyItem()){
        programNameLabel.setText(
            format("{0} - {1}", bodyItem->displayName(), currentProgramItem->displayName()).c_str());
    } else {
        programNameLabel.setText(currentProgramItem->displayName().c_str());
    }

    ExpansionStateMap* pExpansionStateMap = nullptr;
    ProgramState& programState = programStateMap[currentProgramItem];
    pExpansionStateMap = &programState.expansionStateMap;

    if(programState.programConnections.empty()){
        auto programItem = currentProgramItem;
        programState.programConnections.add(
            programItem->sigDisconnectedFromRoot().connect(
                [this, programItem](){ programStateMap.erase(programItem); }));
        
        programState.programConnections.add(
            program->sigStatementRemoved().connect(
                [pExpansionStateMap, programItem](MprStatement* statement, MprProgram*){
                    if(auto structured = dynamic_cast<MprStructuredStatement*>(statement)){
                        pExpansionStateMap->erase(structured);
                    }
                }));
    }

    addStatementsToTree(program, invisibleRootItem(), nullptr, pExpansionStateMap);
}


void MprProgramViewBase::Impl::storeExpansionStates(ExpansionStateMap& expansionStateMap)
{
    storeExpansionStatesIter(currentProgramItem->program(), expansionStateMap);
}


void MprProgramViewBase::Impl::storeExpansionStatesIter
(MprProgram* program, ExpansionStateMap& expansionStateMap)
{
    for(auto& statement : *program){
        if(auto structured = dynamic_cast<MprStructuredStatement*>(statement.get())){
            if(auto item = findStatementItem(structured)){
                expansionStateMap[structured] = item->isExpanded();
                if(auto lowerLevelProgram = structured->lowerLevelProgram()){
                    storeExpansionStatesIter(lowerLevelProgram, expansionStateMap);
                }
            }
        }
    }
}


void MprProgramViewBase::updateStatementTree()
{
    impl->clearStatementTree();

    if(impl->currentProgramItem){
        impl->addStatementsToTree(
            impl->currentProgramItem->program(), impl->invisibleRootItem(), nullptr, nullptr);
    }
}


void MprProgramViewBase::Impl::clearStatementTree()
{
    clear();
    statementItemMap.clear();
}    


void MprProgramViewBase::Impl::addStatementsToTree
(MprProgram* program, QTreeWidgetItem* parentItem, MprStructuredStatement* parentStatement,
 ExpansionStateMap* expansionStateMap)
{
    auto counter = scopedCounterOfStatementItemOperationCall();

    if(parentStatement){
        bool done = false;
        if(expansionStateMap){
            auto p = expansionStateMap->find(parentStatement);
            if(p != expansionStateMap->end()){
                parentItem->setExpanded(p->second);
                done = true;
            }
        }
        if(!done){
            parentItem->setExpanded(parentStatement->isExpandedByDefault());
        }
    }

    if(program->empty()){
        if(program->isSubProgram()){
            // Keep at least one dummy statement item in a sub program
            parentItem->addChild(new StatementItem(dummyStatement, nullptr, this));
        }
    } else {
        for(auto& statement : *program){
            auto statementItem = new StatementItem(statement, program, this);
            parentItem->addChild(statementItem);
            if(auto structured = dynamic_cast<MprStructuredStatement*>(statement.get())){
                if(auto lowerLevelProgram = structured->lowerLevelProgram()){
                    addStatementsToTree(
                        lowerLevelProgram, statementItem, structured, expansionStateMap);
                }
            }
        }
    }
}


MprProgramItemBase* MprProgramViewBase::currentProgramItem()
{
    return impl->currentProgramItem;
}


void MprProgramViewBase::Impl::onItemSelectionChanged()
{
    auto prevCurrentStatement = currentStatement;
    currentStatement.reset();
    auto errorStatement = weak_errorStatement.lock();
    bool isErrorStatementSelected = false;
    
    selectedStatements.clear();
    for(auto& item : selectedItems()){
        auto statementItem = static_cast<StatementItem*>(item);
        auto statement = statementItem->statement();
        if(statement == errorStatement){
            isErrorStatementSelected = true;
        }
        selectedStatements.push_back(statement);
    }

    if(isErrorStatementSelected){
        setStyleSheet("QTreeView::item:selected { background-color: red; } "
                      "QTreeView::branch:selected { background-color: red; }");
    } else {
        setStyleSheet("");
    }

    if(!selectedStatements.empty()){
        currentStatement = selectedStatements.front();
    }

    sigSelectedStatementsChanged(selectedStatements);

    if(currentStatement != prevCurrentStatement){
        self->onCurrentStatementChanged(currentStatement);
        sigCurrentStatementChanged(currentStatement);
    }
}


const std::vector<MprStatementPtr>& MprProgramViewBase::selectedStatements()
{
    return impl->selectedStatements;
}


SignalProxy<void(std::vector<MprStatementPtr>& statements)> MprProgramViewBase::sigSelectedStatementsChanged()
{
    return impl->sigSelectedStatementsChanged;
}


MprStatement* MprProgramViewBase::currentStatement()
{
    return impl->currentStatement;
}


SignalProxy<void(MprStatement* statement)> MprProgramViewBase::sigCurrentStatementChanged()
{
    return impl->sigCurrentStatementChanged;
}


void MprProgramViewBase::Impl::setCurrentStatement(MprStatement* statement)
{
    if(auto item = findStatementItem(statement)){
        setCurrentItem(item);
        scrollToItem(item);
    }
}


void MprProgramViewBase::Impl::setErrorStatement(MprStatement* statement)
{
    auto errorStatement = weak_errorStatement.lock();
    if(statement != errorStatement){
        weak_errorStatement = statement;
        if(errorStatement){
            if(auto item = findStatementItem(errorStatement)){
                QBrush brush;
                for(int i=0; i < NumColumns; ++i){
                    item->setForeground(i, brush);
                }
            }
            if(errorStatement == currentStatement && !statement){
                setStyleSheet(""); // cancel highlight
            }
        }
        if(statement){
            if(auto item = findStatementItem(statement)){
                QBrush brush(Qt::red);
                for(int i=0; i < NumColumns; ++i){
                    item->setForeground(i, brush);
                }
            }
        }
    }
}


void MprProgramViewBase::onCurrentStatementChanged(MprStatement* s)
{

}


// Detect the active (last-clicked) statement
void MprProgramViewBase::Impl::onCurrentTreeWidgetItemChanged
(QTreeWidgetItem* current, QTreeWidgetItem* previous)
{
    if(auto statementItem = dynamic_cast<StatementItem*>(previous)){
        prevActiveStatement = statementItem->statement();
    }
    if(auto statementItem = dynamic_cast<StatementItem*>(current)){
        self->onStatementActivated(statementItem->statement());
    }
}


void MprProgramViewBase::Impl::onTreeWidgetItemClicked(QTreeWidgetItem* item, int /* column */)
{
    if(!isJustAfterDoubleClicked){
        if(auto statementItem = dynamic_cast<StatementItem*>(item)){
            auto statement = statementItem->statement();
            if(statement == prevActiveStatement){
                self->onStatementActivated(statementItem->statement());
            }
            prevActiveStatement = statement;
        }
    } else {
        isJustAfterDoubleClicked = false;
    }
}


void MprProgramViewBase::onStatementActivated(MprStatement* statement)
{
    if(!impl->isOnTimeChanged){
        if(auto ps = dynamic_cast<MprPositionStatement*>(statement)){
            if(impl->bodySyncMode == DirectBodySync){
                impl->currentProgramItem->moveTo(ps);
            } else if(impl->bodySyncMode == TwoStageBodySync){
                impl->currentProgramItem->superimposePosition(ps);
            }
        }
    }
}


void MprProgramViewBase::Impl::onTreeWidgetItemDoubleClicked(QTreeWidgetItem* item, int /* column */)
{
    if(auto statementItem = dynamic_cast<StatementItem*>(item)){
        self->onStatementDoubleClicked(statementItem->statement());
    }
    isJustAfterDoubleClicked = true;
}


void MprProgramViewBase::onStatementDoubleClicked(MprStatement* statement)
{
    if(impl->bodySyncMode == TwoStageBodySync){
        if(auto ps = dynamic_cast<MprPositionStatement*>(statement)){
            impl->currentProgramItem->moveTo(ps);
        }
    }
}
    

MprStatement* MprProgramViewBase::Impl::findStatementAtHierachicalPosition(const vector<int>& position)
{
    if(!position.empty()){
        return findStatementAtHierachicalPositionIter(position, currentProgramItem->program(), 0);
    }
    return nullptr;
}


MprStatement* MprProgramViewBase::Impl::findStatementAtHierachicalPositionIter
(const vector<int>& position, MprProgram* program, int level)
{
    int statementIndex = position[level];
    if(statementIndex < program->numStatements()){
        auto iter = program->begin() + statementIndex;
        auto statement = *iter;
        if(++level == position.size()){
            return statement;
        } else {
            if(auto lower = statement->getLowerLevelProgram()){
                return findStatementAtHierachicalPositionIter(position, lower, level);
            }
        }
    }
    return nullptr;
}


bool MprProgramViewBase::Impl::findControllerItemAndLogItem
(MprControllerItemBase*& controllerItem, ControllerLogItem*& logItem)
{
    if(currentProgramItem){
        controllerItem = currentProgramItem->findOwnerItem<MprControllerItemBase>();
        if(controllerItem){
            logItem = controllerItem->findItem<ControllerLogItem>();
            if(logItem){
                return true;
            }
        }
    }
    return false;
}


shared_ptr<ReferencedObjectSeq> MprProgramViewBase::Impl::findLogSeq()
{
    MprControllerItemBase* controllerItem;
    ControllerLogItem* logItem;
    if(findControllerItemAndLogItem(controllerItem, logItem)){
        return logItem->seq();
    }
    return nullptr;
}
    

bool MprProgramViewBase::Impl::onTimeChanged(double time)
{
    //! \todo Store the controllerItem and logItem in advance to avoid searching them every time
    bool hit = false;

    MprControllerItemBase* controllerItem;
    ControllerLogItem* logItem;

    if(findControllerItemAndLogItem(controllerItem, logItem)){
        auto seq = logItem->seq();
        if(!seq->empty() && seq != invalidLogSeq.lock()){
            auto data = seq->at(seq->lastFrameOfTime(time)).get();
            if(auto log = dynamic_cast<MprControllerLog*>(data)){
                isOnTimeChanged = true;
                if(seekToLogPosition(controllerItem, log)){
                    if(time < seq->timeLength()){
                        hit = true;
                    }
                }
                isOnTimeChanged = false;
            }
        }
    }
    return hit;
}


bool MprProgramViewBase::Impl::seekToLogPosition
(MprControllerItemBase* controllerItem, MprControllerLog* log)
{
    bool result = false;
    
    auto pProgramName = log->sharedTopLevelProgramName();
    if(pProgramName != logTopLevelProgramName){
        MprProgramItemBase* logProgramItem = nullptr;
        auto programItems = controllerItem->descendantItems<MprProgramItemBase>();
        for(auto& programItem : programItems){
            if(!logProgramItem && programItem->name() == *pProgramName){
                logProgramItem = programItem;
                programItem->setSelected(true);
            } else {
                programItem->setSelected(false);
            }
        }
        if(logProgramItem){
            setProgramItem(logProgramItem);
            logTopLevelProgramName = pProgramName;
        }
    }
    if(auto statement = findStatementAtHierachicalPosition(log->hierachicalPosition())){
        if(log->isErrorState()){
            setErrorStatement(statement);
        } else {
            setErrorStatement(nullptr);
        }
        while(auto item = findStatementItem(statement)){
            if(auto parentItem = item->parent()){
                if(!parentItem->isExpanded()){
                    statement = statement->holderProgram()->holderStatement();
                    continue;
                }
            }
            break;
        }

        setCurrentStatement(statement);
    }

    return result;
}


StatementItem* MprProgramViewBase::Impl::findStatementItem(MprStatement* statement)
{
    auto iter = statementItemMap.find(statement);
    if(iter != statementItemMap.end()){
        return iter->second;
    }
    return nullptr;
}


bool MprProgramViewBase::insertStatement(MprStatement* statement, int insertionType)
{   
    return impl->insertStatement(statement, insertionType);
}


bool MprProgramViewBase::Impl::insertStatement(MprStatement* statement, int insertionType)
{
    if(!currentProgramItem){
        return false;
    }

    auto program = currentProgramItem->program();
    MprProgram::iterator pos;
    MprStructuredStatement* parentStatement = nullptr;
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
        if(lastSelectedItem->statement() == dummyStatement){
            pos = program->begin();
        } else  {
            auto index = indexFromItem(lastSelectedItem);
            pos = program->begin() + index.row();
            if(insertionType == AfterTargetPosition){
                ++pos;
            }
        }
    }
    auto inserted = program->insert(pos, statement);

    if(insertionType == AfterTargetPosition){
        clearSelection();
        setCurrentItem(findStatementItem(statement));
    }

    return true;
}


void MprProgramViewBase::Impl::onStatementInserted(MprProgram::iterator iter)
{
    auto counter = scopedCounterOfStatementItemOperationCall();

    auto statement = *iter;
    auto program = statement->holderProgram();
    auto holderStatement = program->holderStatement();

    QTreeWidgetItem* parentItem;
    if(!holderStatement){
        parentItem = invisibleRootItem();
    } else {
        // Check the dummy statement and remove it
        parentItem = findStatementItem(holderStatement);
        if(parentItem->childCount() == 1){
            auto statementItem = static_cast<StatementItem*>(parentItem->child(0));
            if(statementItem->statement() == dummyStatement){
                parentItem->removeChild(statementItem);
                delete statementItem;
            }
        }
    }

    auto statementItem = new StatementItem(statement, program, this);
    bool added = false;
    auto nextIter = ++iter;

    if(nextIter == program->end()){
        parentItem->addChild(statementItem);
        added = true;
    } else {
        if(auto nextItem = findStatementItem(*nextIter)){
            parentItem->insertChild(parentItem->indexOfChild(nextItem), statementItem);
            added = true;
        }
    }

    if(added){
        if(auto structured = dynamic_cast<MprStructuredStatement*>(statement.get())){
            if(auto lowerLevelProgram = structured->lowerLevelProgram()){
                addStatementsToTree(lowerLevelProgram, statementItem, structured, nullptr);
            }
        }
        invalidLogSeq = findLogSeq();
    }
}


void MprProgramViewBase::Impl::onStatementRemoved
(MprProgram* program, MprStatement* statement)
{
    auto counter = scopedCounterOfStatementItemOperationCall();
    
    auto iter = statementItemMap.find(statement);
    if(iter != statementItemMap.end()){
        auto statementItem = iter->second;
        QTreeWidgetItem* parentItem;
        auto holderStatement = program->holderStatement();
        if(holderStatement){
            parentItem = findStatementItem(holderStatement);
        } else {
            parentItem = invisibleRootItem();
        }
        parentItem->removeChild(statementItem);
        delete statementItem;

        if(holderStatement && holderStatement->lowerLevelProgram()->empty()){
            // Keep at least one dummy statement item in a sub program
            parentItem->addChild(new StatementItem(dummyStatement, nullptr, this));
        }

        invalidLogSeq = findLogSeq();
    }
}


void MprProgramViewBase::Impl::onStatementUpdated(MprStatement* statement)
{
    auto iter = statementItemMap.find(statement);
    if(iter != statementItemMap.end()){
        if(statement == weak_errorStatement.lock()){
            setErrorStatement(nullptr);
        }
        invalidLogSeq = findLogSeq();
        auto statementItem = iter->second;
        Q_EMIT dataChanged(indexFromItem(statementItem), indexFromItem(statementItem, NumColumns - 1));
    }
}


void MprProgramViewBase::Impl::forEachStatementInTreeEditEvent
(const QModelIndex& parent, int start, int end,
 function<void(MprStructuredStatement* parentStatement, MprProgram* program,
               int index, MprStatement* statement)> func)
{
    if(!currentProgramItem || isDoingStatementItemOperation()){
        return;
    }
    QTreeWidgetItem* parentItem = nullptr;
    MprStructuredStatement* parentStatement = nullptr;
    MprProgram* program = nullptr;
    
    if(!parent.isValid()){
        parentItem = invisibleRootItem();
        program = currentProgramItem->program();
    } else {
        parentItem = itemFromIndex(parent);
        auto parentStatementItem = static_cast<StatementItem*>(parentItem);
        parentStatement = parentStatementItem->statement<MprStructuredStatement>();
        if(parentStatement){
            program = parentStatement->lowerLevelProgram();
        }
    }
    if(program){
        for(int i = start; i <= end; ++i){
            auto item = static_cast<StatementItem*>(parentItem->child(i));
            func(parentStatement, program, i, item->statement());
        }
    }
}
    

void MprProgramViewBase::Impl::onRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end)
{
    if(isDoingStatementItemOperation()){
        return;
    }

    forEachStatementInTreeEditEvent(
        parent, start, end,
        [&](MprStructuredStatement*, MprProgram* program, int, MprStatement* statement){
            programConnections.block();
            program->remove(statement);
            programConnections.unblock();
        });
}


void MprProgramViewBase::Impl::onRowsRemoved(const QModelIndex& parent, int start, int end)
{
    if(isDoingStatementItemOperation()){
        return;
    }

    // Try to keep at least one dummy statement in a control structure statement
    if(parent.isValid()){
        auto parentItem = itemFromIndex(parent);
        if(auto structured = parentItem->statement<MprStructuredStatement>()){
            auto program = structured->lowerLevelProgram();
            if(program->empty()){
                auto counter = scopedCounterOfStatementItemOperationCall();
                parentItem->addChild(new StatementItem(dummyStatement, nullptr, this));
            }
        }
    }
}


void MprProgramViewBase::Impl::onRowsInserted(const QModelIndex& parent, int start, int end)
{
    if(isDoingStatementItemOperation()){
        return;
    }

    // Check the dummy statement and remove it
    if(parent.isValid()){
        auto parentItem = itemFromIndex(parent);
        std::array<int, 2> rows { start - 1, end + 1 };
        for(auto row : rows){
            if(row >= 0 && row < parentItem->childCount()){
                auto statementItem = static_cast<StatementItem*>(parentItem->child(row));
                if(statementItem->statement() == dummyStatement){
                    auto counter = scopedCounterOfStatementItemOperationCall();
                    parentItem->removeChild(statementItem);
                    delete statementItem;
                    if(row < start){
                        --start;
                        --end;
                    }
                    break;
                }
            }
        }
    }
    
    forEachStatementInTreeEditEvent(
        parent, start, end,
        [&](MprStructuredStatement*, MprProgram* program, int index, MprStatement* statement){
            programConnections.block();
            program->insert(program->begin() + index, statement);
            programConnections.unblock();
        });
}


void MprProgramViewBase::Impl::keyPressEvent(QKeyEvent* event)
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
        default:
            processed = false;
            break;
        }
    }

    if(!processed){
        TreeWidget::keyPressEvent(event);
    }
}            


void MprProgramViewBase::Impl::mousePressEvent(QMouseEvent* event)
{
    TreeWidget::mousePressEvent(event);

    MprStatement* statement = nullptr;
    if(auto statementItem = dynamic_cast<StatementItem*>(itemAt(event->pos()))){
        statement = statementItem->statement();
    }
    
    if(event->button() == Qt::RightButton){
        showContextMenu(statement, event->globalPos());
    }
}


void MprProgramViewBase::Impl::showContextMenu(MprStatement* statement, QPoint globalPos)
{
    contextMenuManager.setNewPopupMenu(this);
    
    if(statement){
        contextMenuFunctions.dispatch(statement);
    } else {
        setBaseContextMenu(contextMenuManager);
    }

    contextMenuManager.popupMenu()->popup(globalPos);
}


void MprProgramViewBase::Impl::setBaseContextMenu(MenuManager& menuManager)
{
    menuManager.addItem(_("Cut"))
        ->sigTriggered().connect([=](){ copySelectedStatements(true); });

    menuManager.addItem(_("Copy"))
        ->sigTriggered().connect([=](){ copySelectedStatements(false); });

    auto pasteAction = menuManager.addItem(_("Paste"));
    if(statementsToPaste.empty()){
        pasteAction->setEnabled(false);
    } else {
        pasteAction->sigTriggered().connect([=](){ pasteStatements(); });
    }

    menuManager.addSeparator();

    menuManager.addItem(_("Insert empty line"))
        ->sigTriggered().connect([=](){ insertStatement(new MprEmptyStatement, BeforeTargetPosition); });
}


void MprProgramViewBase::customizeContextMenu_
(const std::type_info& type,
 std::function<void(MprStatement* statement, MenuManager& menuManager,
                    MprStatementFunctionDispatcher menuFunction)> func)
{
    impl->contextMenuFunctions.setFunction(
        type,
        [this, func](MprStatement* statement){
            func(statement, impl->contextMenuManager, impl->contextMenuFunctions.dispatcher()); }
        );
}


void MprProgramViewBase::Impl::copySelectedStatements(bool doCut)
{
    vector<StatementItem*> selectedStatementTops;
    auto selectedItems_ = selectedItems();
    for(auto& item : selectedItems_){
        auto statementItem = static_cast<StatementItem*>(item);
        if(statementItem->statement() == dummyStatement){
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
        statementsToPaste.clear();
        for(auto& statementItem : selectedStatementTops){
            auto statement = statementItem->statement();
            statementsToPaste.push_back(statement->clone());

            if(doCut){
                MprProgram* program;
                auto parentStatement = statementItem->getParentStatement();
                if(parentStatement){
                    program = parentStatement->lowerLevelProgram();
                } else {
                    program = currentProgramItem->program();
                }
                program->remove(statement);
            }
        }
    }
}


void MprProgramViewBase::Impl::pasteStatements()
{
    MprProgram* program = currentProgramItem->program();
    StatementItem* pastePositionItem = nullptr;
    MprStructuredStatement* parentStatement = nullptr;
    MprProgram::iterator pos;
    
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
        pos = program->begin() + row + 1;
        if(pastePositionItem->statement() == dummyStatement){
            --pos;
        }
    }
    
    for(auto& statement : statementsToPaste){
        auto pasted = statement->clone();
        pos = program->insert(pos, pasted);
        currentProgramItem->resolveStatementReferences(pasted);
        ++pos;
    }
}


bool MprProgramViewBase::storeState(Archive& archive)
{
    impl->targetItemPicker.storeTargetItem(archive, "current_program");
    return true;
}


bool MprProgramViewBase::restoreState(const Archive& archive)
{
    impl->targetItemPicker.restoreTargetItemLater(archive, "current_program");
    return true;
}
