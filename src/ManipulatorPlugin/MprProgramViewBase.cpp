#include "MprProgramViewBase.h"
#include "MprProgramItemBase.h"
#include "MprProgram.h"
#include "BasicMprStatements.h"
#include "MprControllerItemBase.h"
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/TargetItemPicker>
#include <cnoid/TreeWidget>
#include <cnoid/MessageView>
#include <cnoid/TimeBar>
#include <cnoid/BodyItem>
#include <cnoid/BodySuperimposerItem>
#include <cnoid/LinkKinematicsKit>
#include <cnoid/BodyState>
#include <cnoid/ReferencedObjectSeqItem>
#include <cnoid/StringListComboBox>
#include <cnoid/Buttons>
#include <cnoid/ConnectionSet>
#include <cnoid/Archive>
#include <QBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QProxyStyle>
#include <QStyledItemDelegate>
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

    StatementItem(MprStatement* statement, MprProgramViewBase::Impl* viewImpl);
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
    MprProgramItemBasePtr programItem;
    shared_ptr<string> logTopLevelProgramName;
    unordered_map<MprStatementPtr, StatementItem*> statementItemMap;
    MprDummyStatementPtr dummyStatement;
    int statementItemOperationCallCounter;
    Connection currentItemChangeConnection;
    ScopedConnectionSet programConnections;
    MprStatementPtr currentStatement;
    Signal<void(MprStatement* statement)> sigCurrentStatementChanged;
    MprStatementPtr prevCurrentStatement;
    vector<MprStatementPtr> statementsToPaste;

    BodySyncMode bodySyncMode;
    BodySuperimposerItemPtr bodySuperimposer;
    
    QLabel programNameLabel;
    QHBoxLayout buttonBox[3];
    ProgramViewDelegate* mainDelegate;
    ref_ptr<StatementDelegate> defaultStatementDelegate;
    unordered_map<type_index, ref_ptr<StatementDelegate>> statementDelegateMap;
    ToolButton optionMenuButton;
    MenuManager optionMenuManager;

    MenuManager contextMenuManager;
    PolymorphicMprStatementFunctionSet contextMenuFunctions;

    Impl(MprProgramViewBase* self);
    ~Impl();
    void setupWidgets();
    StatementDelegate* findStatementDelegate(MprStatement* statement);
    void onOptionMenuClicked();
    ScopedCounter scopedCounterOfStatementItemOperationCall();
    bool isDoingStatementItemOperation() const;
    void setProgramItem(MprProgramItemBase* item);
    void updateStatementTree();
    void addProgramStatementsToTree(MprProgram* program, QTreeWidgetItem* parentItem);
    void onCurrentTreeWidgetItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous);
    void setCurrentStatement(MprStatement* statement, bool doSetCurrentItem, bool doActivate);
    void onTreeWidgetItemClicked(QTreeWidgetItem* item, int /* column */);
    void onTreeWidgetItemDoubleClicked(QTreeWidgetItem* item, int /* column */);
    MprStatement* findStatementAtHierachicalPosition(const vector<int>& position);
    MprStatement* findStatementAtHierachicalPositionIter(
        const vector<int>& position, MprProgram* program, int level);
    bool onTimeChanged(double time);
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
    bool updateBodyPositionWithPositionStatement(
        MprPositionStatement* ps, bool doUpdateCurrentCoordinateFrames, bool doNotifyKinematicStateChange);
    void initializeBodySuperimposer(BodyItem* bodyItem);
    void superimposePosition(MprPositionStatement* ps);

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


StatementItem::StatementItem(MprStatement* statement_, MprProgramViewBase::Impl* viewImpl)
    : statement_(statement_),
      viewImpl(viewImpl)
{
    if(statement_ != viewImpl->dummyStatement){
        viewImpl->statementItemMap[statement_] = this;
    }
    delegate = viewImpl->findStatementDelegate(statement_);
    
    Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsDragEnabled | Qt::ItemIsEditable;
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
            if(!label.isEmpty()){
                // Add the margin
                if(column == 0){
                    label.append("     ");
                } else {
                    label.append("     ");
                }
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
    if(viewImpl->programItem){
        return viewImpl->programItem->program();
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

    dummyStatement = new MprDummyStatement;

    statementItemOperationCallCounter = 0;

    targetItemPicker.sigTargetItemChanged().connect(
        [&](MprProgramItemBase* item){ setProgramItem(item); });

    defaultStatementDelegate = new StatementDelegate;

    TimeBar::instance()->sigTimeChanged().connect(
        [&](double time){ return onTimeChanged(time); });

    bodySyncMode = DirectBodySync;
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

    currentItemChangeConnection =
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
    impl->bodySyncMode = mode;
}


MprProgramViewBase::BodySyncMode MprProgramViewBase::bodySyncMode() const
{
    return impl->bodySyncMode;
}


void MprProgramViewBase::addEditButton(QWidget* button, int row)
{
    impl->buttonBox[row].addWidget(button);
}


void MprProgramViewBase::onDeactivated()
{
    impl->currentStatement = nullptr;
    impl->prevCurrentStatement = nullptr;
}


void MprProgramViewBase::Impl::onOptionMenuClicked()
{
    optionMenuManager.setNewPopupMenu();
    self->onOptionMenuRequest(optionMenuManager);
    optionMenuManager.popupMenu()->popup(optionMenuButton.mapToGlobal(QPoint(0,0)));
}


void MprProgramViewBase::onOptionMenuRequest(MenuManager& menuManager)
{
    menuManager.addItem(_("Refresh"))->sigTriggered().connect(
        [&](){ updateStatementTree(); });

    menuManager.addItem(_("Renumbering"))->sigTriggered().connect(
        [&](){
            if(impl->programItem){
                impl->programItem->program()->renumberPositionIds();
                impl->programItem->notifyUpdate();
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


void MprProgramViewBase::Impl::setProgramItem(MprProgramItemBase* item)
{
    programConnections.disconnect();
    programItem = item;
    logTopLevelProgramName.reset();
    currentStatement = nullptr;

    if(bodySyncMode == TwoStageBodySync){
        if(bodySuperimposer){
            bodySuperimposer->clearSuperimposition();
            bodySuperimposer.reset();
        }
    }

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
                [&](MprProgram::iterator iter){
                    onStatementInserted(iter); }));

        programConnections.add(
            program->sigStatementRemoved().connect(
                [&](MprProgram* program, MprStatement* statement){
                    onStatementRemoved(program, statement); }));

        programConnections.add(
            program->sigStatementUpdated().connect(
                [&](MprStatement* statement){
                    onStatementUpdated(statement); }));
        
        programNameLabel.setStyleSheet("font-weight: bold");
        programNameLabel.setText(programItem->name().c_str());

        if(bodySyncMode == TwoStageBodySync){
            auto bodyItem = programItem->targetBodyItem();
            if(bodyItem){
                initializeBodySuperimposer(bodyItem);
            }
        }
    }

    updateStatementTree();
}


void MprProgramViewBase::updateStatementTree()
{
    impl->updateStatementTree();
}


void MprProgramViewBase::Impl::updateStatementTree()
{
    clear();
    statementItemMap.clear();
    
    if(programItem){
        addProgramStatementsToTree(programItem->program(), invisibleRootItem());
    }
}


void MprProgramViewBase::Impl::addProgramStatementsToTree
(MprProgram* program, QTreeWidgetItem* parentItem)
{
    auto counter = scopedCounterOfStatementItemOperationCall();

    parentItem->setExpanded(true);

    if(program->empty()){
        if(program->isSubProgram()){
            // Keep at least one dummy statement item in a sub program
            parentItem->addChild(new StatementItem(dummyStatement, this));
        }
    } else {
        for(auto& statement : *program){
            auto statementItem = new StatementItem(statement, this);
            parentItem->addChild(statementItem);
            if(auto structured = dynamic_cast<MprStructuredStatement*>(statement.get())){
                if(auto lowerLevelProgram = structured->lowerLevelProgram()){
                    addProgramStatementsToTree(lowerLevelProgram, statementItem);
                }
            }
        }
    }
}


MprProgramItemBase* MprProgramViewBase::currentProgramItem()
{
    return impl->programItem;
}


MprStatement* MprProgramViewBase::currentStatement()
{
    return impl->currentStatement;
}


SignalProxy<void(MprStatement* statement)> MprProgramViewBase::sigCurrentStatementChanged()
{
    return impl->sigCurrentStatementChanged;
}


void MprProgramViewBase::Impl::onCurrentTreeWidgetItemChanged
(QTreeWidgetItem* current, QTreeWidgetItem* previous)
{
    if(auto statementItem = dynamic_cast<StatementItem*>(previous)){
        prevCurrentStatement = statementItem->statement();
    }
    if(auto statementItem = dynamic_cast<StatementItem*>(current)){
        setCurrentStatement(statementItem->statement(), false, true);
    }
}


void MprProgramViewBase::Impl::setCurrentStatement
(MprStatement* statement, bool doSetCurrentItem, bool doActivate)
{
    if(doSetCurrentItem){
        if(auto item = findStatementItem(statement)){
            currentItemChangeConnection.block();
            setCurrentItem(item);
            currentItemChangeConnection.unblock();
        }
        prevCurrentStatement = statement;
    }
    
    currentStatement = statement;
    self->onCurrentStatementChanged(statement);
    sigCurrentStatementChanged(statement);

    if(doActivate){
        self->onStatementActivated(statement);
    }
}


void MprProgramViewBase::onCurrentStatementChanged(MprStatement*)
{

}


void MprProgramViewBase::Impl::onTreeWidgetItemClicked(QTreeWidgetItem* item, int /* column */)
{
    if(auto statementItem = dynamic_cast<StatementItem*>(item)){
        auto statement = statementItem->statement();
        // If the clicked statement is different from the current one,
        // onCurrentTreeWidgetItemChanged is processed
        if(statement == prevCurrentStatement){
            self->onStatementActivated(statement);
        }
        prevCurrentStatement = statement;
    }
}


void MprProgramViewBase::onStatementActivated(MprStatement* statement)
{
    if(auto ps = dynamic_cast<MprPositionStatement*>(statement)){
        if(impl->bodySyncMode == DirectBodySync){
            impl->updateBodyPositionWithPositionStatement(ps, true, true);
        } else if(impl->bodySyncMode == TwoStageBodySync){
            impl->superimposePosition(ps);
        }
    }
}


void MprProgramViewBase::Impl::onTreeWidgetItemDoubleClicked(QTreeWidgetItem* item, int /* column */)
{
    if(auto statementItem = dynamic_cast<StatementItem*>(item)){
        auto statement = statementItem->statement();
        self->onStatementDoubleClicked(statement);
        prevCurrentStatement = nullptr;
    }
}


void MprProgramViewBase::onStatementDoubleClicked(MprStatement* statement)
{
    if(impl->bodySyncMode == TwoStageBodySync){
        if(auto ps = dynamic_cast<MprPositionStatement*>(statement)){
            impl->updateBodyPositionWithPositionStatement(ps, true, true);
        }
    }
}
    

MprStatement* MprProgramViewBase::Impl::findStatementAtHierachicalPosition(const vector<int>& position)
{
    if(!position.empty()){
        return findStatementAtHierachicalPositionIter(position, programItem->program(), 0);
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


bool MprProgramViewBase::Impl::onTimeChanged(double time)
{
    //! \todo Store the controllerItem and logItem in advance to avoid searching them every time
    bool hit = false;
    if(programItem){
        if(auto controllerItem = programItem->findOwnerItem<MprControllerItemBase>()){
            if(auto logItem = controllerItem->findItem<ReferencedObjectSeqItem>()){
                auto seq = logItem->seq();
                if(!seq->empty()){
                    auto data = seq->at(seq->lastFrameOfTime(time)).get();
                    if(auto logData = dynamic_cast<MprControllerLog*>(data)){
                        auto& programName = logData->topLevelProgramName;
                        if(programName != logTopLevelProgramName){
                            if(auto logProgramItem = controllerItem->findItem<MprProgramItemBase>(*programName)){
                                setProgramItem(logProgramItem);
                                logTopLevelProgramName = programName;
                            }
                        }
                        if(auto statement = findStatementAtHierachicalPosition(logData->hierachicalPosition)){
                            setCurrentStatement(statement, true, false);
                            if(time < seq->timeLength()){
                                hit = true;
                            }
                        }
                    }
                }
            }
        }
    }
    return hit;
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
    if(!programItem){
        return false;
    }

    auto program = programItem->program();
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
        findStatementItem(statement)->setSelected(true);
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

    auto statementItem = new StatementItem(statement, this);
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
                addProgramStatementsToTree(lowerLevelProgram, statementItem);
            }
        }
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
            parentItem->addChild(new StatementItem(dummyStatement, this));
        }
    }
}


void MprProgramViewBase::Impl::onStatementUpdated(MprStatement* statement)
{
    auto iter = statementItemMap.find(statement);
    if(iter != statementItemMap.end()){
        auto statementItem = iter->second;
        Q_EMIT dataChanged(indexFromItem(statementItem), indexFromItem(statementItem, NumColumns - 1));
    }
}


void MprProgramViewBase::Impl::forEachStatementInTreeEditEvent
(const QModelIndex& parent, int start, int end,
 function<void(MprStructuredStatement* parentStatement, MprProgram* program,
               int index, MprStatement* statement)> func)
{
    if(!programItem || isDoingStatementItemOperation()){
        return;
    }
    QTreeWidgetItem* parentItem = nullptr;
    MprStructuredStatement* parentStatement = nullptr;
    MprProgram* program = nullptr;
    
    if(!parent.isValid()){
        parentItem = invisibleRootItem();
        program = programItem->program();
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
                parentItem->addChild(new StatementItem(dummyStatement, this));
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
                    program = programItem->program();
                }
                program->remove(statement);
            }
        }
    }
}


void MprProgramViewBase::Impl::pasteStatements()
{
    MprProgram* program = programItem->program();
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
        pos = program->insert(pos, statement->clone());
        ++pos;
    }
}


bool MprProgramViewBase::updateBodyPositionWithPositionStatement
(MprPositionStatement* ps, bool doUpdateCurrentCoordinateFrames, bool doNotifyKinematicStateChange)
{
    return impl->updateBodyPositionWithPositionStatement(
        ps, doUpdateCurrentCoordinateFrames, doNotifyKinematicStateChange);
}


bool MprProgramViewBase::Impl::updateBodyPositionWithPositionStatement
(MprPositionStatement* ps, bool doUpdateCurrentCoordinateFrames, bool doNotifyKinematicStateChange)
{
    bool updated = false;
    if(auto kinematicsKit = programItem->kinematicsKit()){
        auto positions = programItem->program()->positions();
        auto position = ps->position(positions);
        if(!position){
            MessageView::instance()->putln(
                format(_("Position {0} is not found."), ps->positionLabel()), MessageView::WARNING);
        } else {
            updated = position->apply(kinematicsKit);
            if(updated){
                if(doUpdateCurrentCoordinateFrames){
                    if(auto ikPosition = dynamic_cast<MprIkPosition*>(position)){
                        kinematicsKit->setCurrentBaseFrameType(ikPosition->baseFrameType());
                        kinematicsKit->setCurrentBaseFrame(ikPosition->baseFrameId());
                        kinematicsKit->setCurrentEndFrame(ikPosition->toolFrameId());
                        kinematicsKit->notifyCurrentFrameChange();
                    }
                }
                if(doNotifyKinematicStateChange){
                    programItem->targetBodyItem()->notifyKinematicStateChange();
                }
            }
        }
    }
    return updated;
}


void MprProgramViewBase::Impl::initializeBodySuperimposer(BodyItem* bodyItem)
{
    bodySuperimposer = bodyItem->findChildItem<BodySuperimposerItem>("MprPositionSuperimposer");

    if(!bodySuperimposer){
        bodySuperimposer = new BodySuperimposerItem;
        bodySuperimposer->setName("MprPositionSuperimposer");
        bodySuperimposer->setTemporal();
        bodyItem->addChildItem(bodySuperimposer);
    }
}


void MprProgramViewBase::Impl::superimposePosition(MprPositionStatement* ps)
{
    if(bodySuperimposer){
        auto orgBody = programItem->targetBodyItem()->body();
        BodyState orgBodyState(*orgBody);

        if(updateBodyPositionWithPositionStatement(ps, false, false)){
            // Main body
            auto superBody = bodySuperimposer->superimposedBody(0);
            BodyState bodyState(*orgBody);
            bodyState.restorePositions(*superBody);
            superBody->calcForwardKinematics();

            // Child bodies
            const int n = bodySuperimposer->numSuperimposedBodies();
            for(int i=1; i < n; ++i){
                auto childBody = bodySuperimposer->superimposedBody(i);
                childBody->syncPositionWithParentBody();
            }
            
            bodySuperimposer->updateSuperimposition();
            orgBodyState.restorePositions(*orgBody);
        }
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
