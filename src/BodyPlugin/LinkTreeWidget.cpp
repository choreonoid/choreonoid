/**
   @author Shin'ichiro Nakaoka
*/

#include "LinkTreeWidget.h"
#include <cnoid/Link>
#include <cnoid/LinkGroup>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <cnoid/MenuManager>
#include <cnoid/ItemManager>
#include <QHeaderView>
#include <QCheckBox>
#include <QRadioButton>
#include <QBoxLayout>
#include <QEvent>
#include <QApplication>
#include <set>
#include <map>
#include <deque>
#include <iostream>
#include <cassert>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {
const bool TRACE_FUNCTIONS = false;
}

namespace cnoid {

class LinkTreeWidgetImpl
{
public:
    LinkTreeWidgetImpl(LinkTreeWidget* self);
    ~LinkTreeWidgetImpl();

    LinkTreeWidget* self;

    struct ColumnInfo {
        LinkTreeWidget::ColumnDataFunction dataFunction;
        LinkTreeWidget::ColumnSetDataFunction setDataFunction;
        LinkTreeWidget::ColumnWidgetFunction widgetFunction;
    };
    vector<ColumnInfo> columnInfos;

    QTreeWidgetItem* headerItem;
    int nameColumn;
    int jointIdColumn;
    int rowIndexCounter;
    int itemWidgetWidthAdjustment;
    vector<LinkTreeItem*> customRows;
    bool isNameColumnMarginEnabled;
    LinkTreeWidget::ListingMode listingMode;
    ComboBox listingModeCombo;
    Menu popupMenu; // This must be defined before popupMenuManager
    MenuManager popupMenuManager;

    Signal<void(bool isInitialCreation)> sigUpdateRequest;
    Signal<void(LinkTreeItem* item, int column)> sigItemChanged;
    Signal<void()> sigSelectionChanged;

    int defaultExpansionLevel;
    bool isCacheEnabled;
    bool isArchiveOfCurrentBodyItemEnabled;

    class BodyItemInfo {
    public:
        bool isRestoringTreeStateNeeded;

        LinkGroupPtr linkGroup;

        vector<bool> selection;
        vector<int> selectedLinkIndices;
        Signal<void()> sigSelectionChanged;

        bool needTreeExpansionUpdate;
        vector<bool> linkExpansions;
        set<string> expandedParts;
                
        Connection detachedFromRootConnection;

        BodyItemInfo() {
            isRestoringTreeStateNeeded = true;
        }
        ~BodyItemInfo() {
            detachedFromRootConnection.disconnect();
        }
        void setNumLinks(int n, bool doInitialize) {
            if(doInitialize){
                selection.clear();
                linkExpansions.clear();
                expandedParts.clear();
            }
            selection.resize(n, false);
            linkExpansions.resize(n, true);
        }
    };
    typedef shared_ptr<BodyItemInfo> BodyItemInfoPtr;

    typedef map<BodyItemPtr, BodyItemInfoPtr> BodyItemInfoMap;
    BodyItemInfoMap bodyItemInfoCache;

    vector<LinkTreeItem*> linkIndexToItemMap;

    BodyItemPtr currentBodyItem;
    BodyItemInfoPtr currentBodyItemInfo;

    Signal<void()> dummySigSelectionChanged; // never emitted
    vector<int> emptyLinkIndices;
    vector<bool> emptySelection;

    void initialize();
    void enableCache(bool on);
    BodyItemInfoPtr getBodyItemInfo(BodyItem* bodyItem);
    void onBodyItemDetachedFromRoot(BodyItem* bodyItem);
    void clearTreeItems();
    void setCurrentBodyItem(BodyItem* bodyItem, bool forceTreeUpdate);
    void restoreTreeState();
    void restoreSubTreeState(QTreeWidgetItem* item);
    void restoreTreeStateSub(QTreeWidgetItem* parentItem);
    void addChild(LinkTreeItem* parentItem, LinkTreeItem* item);
    void addChild(LinkTreeItem* item);
    void setLinkTree(Link* link, bool onlyJoints);
    void setLinkTreeSub(Link* link, Link* parentLink, LinkTreeItem* parentItem, bool onlyJoints);
    void setJointList(const BodyPtr& body);
    void setLinkList(const BodyPtr& body);
    void setLinkGroupTree(const BodyPtr& body, const LinkGroupPtr& linkGroup);
    void setLinkGroupTreeSub(LinkTreeItem* parentItem, const LinkGroupPtr& linkGroup, const BodyPtr& body);
    void addCustomRows();
    void onListingModeChanged(int index);
    void onSelectionChanged();
    Signal<void()>& sigSelectionChangedOf(BodyItem* bodyItem);
    int selectedLinkIndex(BodyItem* bodyItem) const;
    const vector<int>& selectedLinkIndices(BodyItem* bodyItem);
    const vector<bool>& linkSelection(BodyItem* bodyItem);
    void onCustomContextMenuRequested(const QPoint& pos);
    void setExpansionState(const LinkTreeItem* item, bool on);
    void onItemExpanded(QTreeWidgetItem* treeWidgetItem);
    void onItemCollapsed(QTreeWidgetItem* treeWidgetItem);
    bool makeSingleSelection(BodyItem* bodyItem, int linkIndex);
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
};

}

namespace {

void jointIdData(const LinkTreeItem* item, int role, QVariant& out_value)
{
    const Link* link = item->link();
    if(role == Qt::DisplayRole && link && (link->jointId() >= 0)){
        out_value = link->jointId();
    } else if(role == Qt::TextAlignmentRole){
        out_value = Qt::AlignHCenter;
    }
        
}

void nameData(const LinkTreeItem* item, int role, QVariant& out_value)
{
    if(role == Qt::DisplayRole){
        out_value = item->nameText();
    }
}
}


LinkTreeItem::LinkTreeItem(const std::string& name)
    : name_(name),
      nameText_(name.c_str())
{
    rowIndex_ = -1;
    link_ = 0;
    isLinkGroup_ = true;
}


LinkTreeItem::LinkTreeItem(Link* link, LinkTreeWidgetImpl* treeImpl)
    : name_(link->name())
{
    if(treeImpl->isNameColumnMarginEnabled){
        nameText_ = QString(" %1 ").arg(name_.c_str());
    } else {
        nameText_ = name_.c_str();
    }
    rowIndex_ = -1;
    link_ = link;
    isLinkGroup_ = false;
    treeImpl->linkIndexToItemMap[link->index()] = this;
}


LinkTreeItem::LinkTreeItem(LinkGroup* linkGroup, LinkTreeWidgetImpl* treeImpl)
    : name_(linkGroup->name())
{
    if(treeImpl->isNameColumnMarginEnabled){
        nameText_ = QString(" %1 ").arg(name_.c_str());
    } else {
        nameText_ = name_.c_str();
    }
    rowIndex_ = -1;
    link_ = 0;
    isLinkGroup_ = true;
}


QVariant LinkTreeItem::data(int column, int role) const
{
    QVariant value;
    LinkTreeWidget* tree = static_cast<LinkTreeWidget*>(treeWidget());
    LinkTreeWidget::ColumnDataFunction& func = tree->impl->columnInfos[column].dataFunction;
    if(func){
        func(this, role, value);
    }
    if(value.isValid()){
        return value;
    }
    return QTreeWidgetItem::data(column, role);
}


void LinkTreeItem::setData(int column, int role, const QVariant& value)
{
    LinkTreeWidget* tree = static_cast<LinkTreeWidget*>(treeWidget());
    LinkTreeWidget::ColumnSetDataFunction& func = tree->impl->columnInfos[column].setDataFunction;
    if(func){
        func(this, role, value);
    }
    return QTreeWidgetItem::setData(column, role, value);
}


LinkTreeWidget::LinkTreeWidget(QWidget* parent)
    : TreeWidget(parent)
{
    impl = new LinkTreeWidgetImpl(this);
    impl->initialize();
}


LinkTreeWidgetImpl::LinkTreeWidgetImpl(LinkTreeWidget* self)
    : self(self),
      popupMenuManager(&popupMenu)
{

}


void LinkTreeWidgetImpl::initialize()
{
    rowIndexCounter = 0;
    defaultExpansionLevel = std::numeric_limits<int>::max();
    isCacheEnabled = false;
    isArchiveOfCurrentBodyItemEnabled = false;
    itemWidgetWidthAdjustment = 0;
    isNameColumnMarginEnabled = false;
    
    headerItem = new QTreeWidgetItem;
    QHeaderView* header = self->header();
    header->setMinimumSectionSize(0);
    header->setStretchLastSection(false);

    self->sigSectionResized().connect(
        [&](int, int, int){ self->updateGeometry(); });

    self->setHeaderItem(headerItem);
    self->setSelectionMode(QAbstractItemView::ExtendedSelection);
    self->setIndentation(12);
    self->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    self->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    self->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);

    nameColumn = self->addColumn(_("Link"));
    header->setSectionResizeMode(nameColumn, QHeaderView::Stretch);
    self->setColumnDataFunction(nameColumn, &nameData);

    jointIdColumn = self->addColumn(_("ID"));
    self->setColumnDataFunction(jointIdColumn, &jointIdData);
    header->setSectionResizeMode(jointIdColumn, QHeaderView::ResizeToContents);
    
    headerItem->setTextAlignment(jointIdColumn, Qt::AlignCenter);
    self->moveVisualColumnIndex(jointIdColumn, 0);

    QObject::connect(self, SIGNAL(itemChanged(QTreeWidgetItem*, int)),
                     self, SLOT(onItemChanged(QTreeWidgetItem*, int)));

    QObject::connect(self, SIGNAL(itemExpanded(QTreeWidgetItem*)), self,
                     SLOT(onItemExpanded(QTreeWidgetItem*)));

    QObject::connect(self, SIGNAL(itemCollapsed(QTreeWidgetItem*)),
                     self, SLOT(onItemCollapsed(QTreeWidgetItem*)));

    QObject::connect(self, SIGNAL(itemSelectionChanged()),
                     self, SLOT(onSelectionChanged()));

    // The order of the following labes must correspond to that of ListingMode
    listingModeCombo.enableI18n(CNOID_GETTEXT_DOMAIN_NAME);
    listingModeCombo.addI18nItem(N_("Link List"));
    listingModeCombo.addI18nItem(N_("Link Tree"));
    listingModeCombo.addI18nItem(N_("Joint List"));
    listingModeCombo.addI18nItem(N_("Joint Tree"));
    listingModeCombo.addI18nItem(N_("Part Tree"));

    listingMode = LinkTreeWidget::LINK_LIST;
    listingModeCombo.setCurrentIndex(listingMode);
    listingModeCombo.sigCurrentIndexChanged().connect(
        [&](int index){ onListingModeChanged(index); });
}


LinkTreeWidget::~LinkTreeWidget()
{
    delete impl;
}


LinkTreeWidgetImpl::~LinkTreeWidgetImpl()
{
    for(size_t i=0; i < customRows.size(); ++i){
        delete customRows[i];
    }
}


void LinkTreeWidget::setDefaultExpansionLevel(int level)
{
    impl->defaultExpansionLevel = level;
}


void LinkTreeWidget::enableCache(bool on)
{
    impl->enableCache(on);
}


void LinkTreeWidgetImpl::enableCache(bool on)
{
    isCacheEnabled = on;

    if(!isCacheEnabled){
        bodyItemInfoCache.clear();
    }
}


int LinkTreeWidget::nameColumn()
{
    return impl->nameColumn;
}


int LinkTreeWidget::jointIdColumn()
{
    return impl->jointIdColumn;
}


int LinkTreeWidget::setNumColumns(int n)
{
    setColumnCount(n);
    impl->columnInfos.resize(n);
    return 0;
}


int LinkTreeWidget::addColumn()
{
    int column = impl->columnInfos.size();
    impl->columnInfos.push_back(LinkTreeWidgetImpl::ColumnInfo());
    setColumnCount(impl->columnInfos.size());
    impl->headerItem->setText(column, QString());
    header()->setSectionResizeMode(column, QHeaderView::ResizeToContents);
    return column;
}


int LinkTreeWidget::addColumn(const QString& headerText)
{
    int column = addColumn();
    impl->headerItem->setText(column, headerText);
    header()->setSectionResizeMode(column, QHeaderView::ResizeToContents);
    return column;
}


void LinkTreeWidget::setColumnStretchResizeMode(int column)
{
    header()->setSectionResizeMode(column, QHeaderView::Stretch);
}


void LinkTreeWidget::setColumnInteractiveResizeMode(int column)
{
    header()->setSectionResizeMode(column, QHeaderView::Interactive);
}


void LinkTreeWidget::setColumnResizeToContentsMode(int column)
{
    header()->setSectionResizeMode(column, QHeaderView::ResizeToContents);
}

                    
void LinkTreeWidget::moveVisualColumnIndex(int column, int visualIndex)
{
    header()->moveSection(header()->visualIndex(column), visualIndex);
}


void LinkTreeWidget::setColumnDataFunction(int column, ColumnDataFunction func)
{
    impl->columnInfos[column].dataFunction = func;
}


void LinkTreeWidget::setColumnSetDataFunction(int column, ColumnSetDataFunction func)
{
    impl->columnInfos[column].setDataFunction = func;
}


void LinkTreeWidget::setColumnWidgetFunction(int column, ColumnWidgetFunction func)
{
    impl->columnInfos[column].widgetFunction = func;
}


void LinkTreeWidget::changeEvent(QEvent* event)
{
    QTreeWidget::changeEvent(event);
    if(event->type() == QEvent::StyleChange){
        //impl->setCurrentBodyItem(impl->currentBodyItem, true);
    }
}


void LinkTreeWidget::setNameColumnMarginEnabled(bool on)
{
    impl->isNameColumnMarginEnabled = on;
}


void LinkTreeWidget::setAlignedItemWidget
(LinkTreeItem* item, int column, QWidget* widget, Qt::Alignment alignment)
{
    QHBoxLayout* box = new QHBoxLayout();
    box->setContentsMargins(0, 0, 0, 0);
    if(impl->itemWidgetWidthAdjustment > 0){
        widget->setMinimumWidth(widget->sizeHint().width() + impl->itemWidgetWidthAdjustment);
    }
    box->addWidget(widget, 0, alignment);
    QWidget* base = new QWidget();
    base->setLayout(box);
    setItemWidget(item, column, base);
}


QWidget* LinkTreeWidget::alignedItemWidget(LinkTreeItem* item, int column)
{
    QWidget* base = itemWidget(item, column);
    if(base){
        QLayout* layout = base->layout();
        if(layout){
            QLayoutItem* layoutItem = layout->itemAt(0);
            if(layoutItem){
                QWidget* widget = layoutItem->widget();
                if(widget){
                    return widget;
                }
            }
        }
    }
    return 0;
}


void LinkTreeWidget::addCustomRow(LinkTreeItem* treeItem)
{
    impl->customRows.push_back(treeItem);
}


int LinkTreeWidget::numLinkTreeItems()
{
    return impl->rowIndexCounter;
}


SignalProxy<void(bool isInitialCreation)> LinkTreeWidget::sigUpdateRequest()
{
    return impl->sigUpdateRequest;
}


ComboBox* LinkTreeWidget::listingModeCombo()
{
    return &impl->listingModeCombo;
}


void LinkTreeWidget::setListingMode(ListingMode mode)
{
    impl->listingMode = mode;
    impl->listingModeCombo.setCurrentIndex(mode);
}


void LinkTreeWidget::fixListingMode(bool on)
{
    impl->listingModeCombo.setEnabled(on);
}


LinkTreeWidgetImpl::BodyItemInfoPtr LinkTreeWidgetImpl::getBodyItemInfo(BodyItem* bodyItem)
{
    BodyItemInfoPtr info;
    bool isInfoForNewBody = false;

    if(bodyItem){
        if(bodyItem == currentBodyItem){
            info = currentBodyItemInfo;
        } else if(isCacheEnabled){
            auto p = bodyItemInfoCache.find(bodyItem);
            if(p != bodyItemInfoCache.end()){
                info = p->second;
            } else if(isCacheEnabled){
                auto originalItem = ItemManager::findOriginalItemForReloadedItem(bodyItem);
                auto q = bodyItemInfoCache.find(dynamic_cast<BodyItem*>(originalItem));
                if(q != bodyItemInfoCache.end()){
                    info = q->second;
                    isInfoForNewBody = true;
                    bodyItemInfoCache.erase(q);
                }
            }
        }

        if(!info && bodyItem->findRootItem()){
            if(!isCacheEnabled){
                bodyItemInfoCache.clear();
            }
            info = std::make_shared<BodyItemInfo>();
            isInfoForNewBody = true;
        }

        if(isInfoForNewBody){
            info->linkGroup = LinkGroup::create(*bodyItem->body());
            info->detachedFromRootConnection = bodyItem->sigDetachedFromRoot().connect(
                [this, bodyItem](){ onBodyItemDetachedFromRoot(bodyItem); });
            bodyItemInfoCache[bodyItem] = info;
        }

        if(info){
            info->setNumLinks(bodyItem->body()->numLinks(), false);
        }
    }

    return info;
}


void LinkTreeWidgetImpl::onBodyItemDetachedFromRoot(BodyItem* bodyItem)
{
    if(TRACE_FUNCTIONS){
        cout << "LinkTreeWidgetImpl::onBodyItemDetachedFromRoot(" << bodyItem->name() << ")" << endl;
    }
    
    if(currentBodyItem == bodyItem){
        setCurrentBodyItem(0, false);
    }

    BodyItemInfoMap::iterator p = bodyItemInfoCache.find(bodyItem);
    if(p != bodyItemInfoCache.end()){
        p->second->detachedFromRootConnection.disconnect();
        bodyItemInfoCache.erase(p);
    }
}

void LinkTreeWidget::setBodyItem(BodyItem* bodyItem)
{
    impl->setCurrentBodyItem(bodyItem, false);
}


BodyItem* LinkTreeWidget::bodyItem()
{
    return impl->currentBodyItem;
}


LinkTreeItem* LinkTreeWidget::itemOfLink(int linkIndex)
{
    if(linkIndex < (int)impl->linkIndexToItemMap.size()){
        return impl->linkIndexToItemMap[linkIndex];
    }
    return 0;
}


void LinkTreeWidgetImpl::clearTreeItems()
{
    // Take custom row items before calling clear() to prevent the items from being deleted.
    for(size_t i=0; i < customRows.size(); ++i){
        LinkTreeItem* item = customRows[i];
        if(item->treeWidget()){
            self->takeTopLevelItem(self->indexOfTopLevelItem(item));
        }
    }
    self->clear();
}


void LinkTreeWidgetImpl::setCurrentBodyItem(BodyItem* bodyItem, bool forceTreeUpdate)
{
    if(TRACE_FUNCTIONS){
        cout << "LinkTreeWidgetImpl::setCurrentBodyItem(" << (bodyItem ? bodyItem->name() : string("0"))
        << ", " << forceTreeUpdate << ")" << endl;
    }

    bool currentChanged = bodyItem != currentBodyItem;
    
    if(forceTreeUpdate || currentChanged){

        self->blockSignals(true);

        clearTreeItems();
        rowIndexCounter = 0;
        linkIndexToItemMap.clear();

        if(QApplication::style()->objectName() == "cleanlooks"){
            itemWidgetWidthAdjustment = 2;
        } else {
            itemWidgetWidthAdjustment = 0;
        }
        
        self->blockSignals(false);

        currentBodyItemInfo = getBodyItemInfo(bodyItem);
        currentBodyItem = bodyItem;

        if(bodyItem){

            BodyPtr body = bodyItem->body();
            linkIndexToItemMap.resize(body->numLinks(), 0);

            switch(listingMode){
            case LinkTreeWidget::LINK_LIST:
                self->setRootIsDecorated(false);
                setLinkList(body);
                headerItem->setText(nameColumn, _("Link"));
                break;
            case LinkTreeWidget::LINK_TREE:
                self->setRootIsDecorated(true);
                setLinkTree(body->rootLink(), false);
                headerItem->setText(nameColumn, _("Link"));
                break;
            case LinkTreeWidget::JOINT_LIST:
                self->setRootIsDecorated(false);
                setJointList(body);
                headerItem->setText(nameColumn, _("Joint"));
                break;
            case LinkTreeWidget::JOINT_TREE:
                self->setRootIsDecorated(true);
                setLinkTree(body->rootLink(), true);
                headerItem->setText(nameColumn, _("Joint"));
                break;
            case LinkTreeWidget::PART_TREE:
                self->setRootIsDecorated(true);
                setLinkGroupTree(bodyItem->body(), currentBodyItemInfo->linkGroup);
                headerItem->setText(nameColumn, _("Link"));
                break;
            default:
                break;
            }

            addCustomRows();

            restoreTreeState();
        }

        sigUpdateRequest(true);
    }
}


void LinkTreeWidgetImpl::restoreTreeState()
{
    if(TRACE_FUNCTIONS){
        cout << "LinkTreeWidgetImpl::restoreTreeState()" << endl;
    }
    
    if(currentBodyItemInfo){
        restoreSubTreeState(self->invisibleRootItem());
        currentBodyItemInfo->isRestoringTreeStateNeeded = false;
    }
}


void LinkTreeWidgetImpl::restoreSubTreeState(QTreeWidgetItem* item)
{
    self->blockSignals(true);
    restoreTreeStateSub(item);
    self->blockSignals(false);
}


void LinkTreeWidgetImpl::restoreTreeStateSub(QTreeWidgetItem* parentItem)
{
    if(TRACE_FUNCTIONS){
        cout << "LinkTreeWidgetImpl::restoreTreeStateSub()" << endl;
    }

    int n = parentItem->childCount();
    for(int i=0; i < n; ++i){
        LinkTreeItem* item = dynamic_cast<LinkTreeItem*>(parentItem->child(i));
        if(item){
            const Link* link = item->link();
            if(link){
                item->setSelected(currentBodyItemInfo->selection[link->index()]);
            }
        
            if(item->childCount() > 0){ // Tree
            
                bool expanded = item->isExpanded();
        
                if(listingMode == LinkTreeWidget::PART_TREE){
                    if(!link){
                        const std::set<string>& parts = currentBodyItemInfo->expandedParts;                        
                        expanded = (parts.find(item->name()) != parts.end());
                        item->setExpanded(expanded);
                    }
                } else if(link){
                    expanded = currentBodyItemInfo->linkExpansions[link->index()];
                    item->setExpanded(expanded);
                }
                if(expanded){
                    restoreTreeStateSub(item);
                }
            }
        }
    }
}


void LinkTreeWidgetImpl::addChild(LinkTreeItem* parentItem, LinkTreeItem* item)
{
    if(parentItem){
        parentItem->addChild(item);
    } else {
        self->addTopLevelItem(item);
    }
    item->rowIndex_ = rowIndexCounter++;

    for(size_t col=0; col < columnInfos.size(); ++col){
        LinkTreeWidget::ColumnWidgetFunction& func = columnInfos[col].widgetFunction;
        if(func){
            QWidget* widget = func(item);
            if(widget){
                self->setItemWidget(item, col, widget);
            }
        }
    }
}


void LinkTreeWidgetImpl::addChild(LinkTreeItem* item)
{
    addChild(0, item);
}


void LinkTreeWidgetImpl::setLinkTree(Link* link, bool onlyJoints)
{
    self->blockSignals(true);
    setLinkTreeSub(link, 0, 0, onlyJoints);
    self->blockSignals(false);
}


void LinkTreeWidgetImpl::setLinkTreeSub(Link* link, Link* parentLink, LinkTreeItem* parentItem, bool onlyJoints)
{
    LinkTreeItem* item = 0;

    if(onlyJoints && link->jointId() < 0){
        item = parentItem;
    } else {
        item = new LinkTreeItem(link, this);
        addChild(parentItem, item);
        item->setExpanded(true);
    }

    if(link->child()){
        for(Link* child = link->child(); child; child = child->sibling()){
            setLinkTreeSub(child, link, item, onlyJoints);
        }
    }
}


void LinkTreeWidgetImpl::setJointList(const BodyPtr& body)
{
    for(int i=0; i < body->numJoints(); ++i){
        Link* joint = body->joint(i);
        if(joint->isValid()){
            addChild(new LinkTreeItem(joint, this));
        }
    }
}


void LinkTreeWidgetImpl::setLinkList(const BodyPtr& body)
{
    if(TRACE_FUNCTIONS){
        cout << "LinkTreeWidgetImpl::setLinkList(" << body->name() << ")" << endl;
    }
    
    for(int i=0; i < body->numLinks(); ++i){
        addChild(new LinkTreeItem(body->link(i), this));
    }
}


void LinkTreeWidgetImpl::setLinkGroupTree(const BodyPtr& body, const LinkGroupPtr& linkGroup)
{
    if(linkGroup){
        self->blockSignals(true);
        setLinkGroupTreeSub(0, linkGroup, body);
        self->blockSignals(false);
    }
}


void LinkTreeWidgetImpl::setLinkGroupTreeSub(LinkTreeItem* parentItem, const LinkGroupPtr& linkGroup, const BodyPtr& body)
{
    LinkTreeItem* item = new LinkTreeItem(linkGroup.get(), this);
    addChild(parentItem, item);

    int numChildGroups = 0;
    int n = linkGroup->numElements();
    for(int i=0; i < n; ++i){
        if(linkGroup->isSubGroup(i)){
            setLinkGroupTreeSub(item, linkGroup->subGroup(i), body);
            ++numChildGroups;
        } else if(linkGroup->isLinkIndex(i)){
            Link* link = body->link(linkGroup->linkIndex(i));
            if(link){
                addChild(item, new LinkTreeItem(link, this));
            }
        }
    }

    item->setExpanded(numChildGroups > 0);
}


void LinkTreeWidgetImpl::addCustomRows()
{
    for(size_t i=0; i < customRows.size(); ++i){
        addChild(customRows[i]);
    }
}


void LinkTreeWidgetImpl::onListingModeChanged(int index)
{
    LinkTreeWidget::ListingMode newListingMode = static_cast<LinkTreeWidget::ListingMode>(index);
    if(newListingMode != listingMode){
        listingMode = newListingMode;
        if(currentBodyItem){
            setCurrentBodyItem(currentBodyItem, true);
        }
    }
}


void LinkTreeWidget::onSelectionChanged()
{
    impl->onSelectionChanged();
}


void LinkTreeWidgetImpl::onSelectionChanged()
{
    if(TRACE_FUNCTIONS){
        cout << "LinkTreeWidgetImpl::onSelectionChanged()" << endl;
    }

    if(currentBodyItem){
        auto& selection = currentBodyItemInfo->selection;
        std::fill(selection.begin(), selection.end(), false);
        QList<QTreeWidgetItem*> selected = self->selectedItems();
        for(int i=0; i < selected.size(); ++i){
            LinkTreeItem* item = dynamic_cast<LinkTreeItem*>(selected[i]);
            if(item && item->link()){
                selection[item->link()->index()] = true;
            }
        }
        currentBodyItemInfo->sigSelectionChanged();
        sigSelectionChanged();
    }
}


SignalProxy<void(LinkTreeItem* item, int column)> LinkTreeWidget::sigItemChanged()
{
    return impl->sigItemChanged;
}


void LinkTreeWidget::onItemChanged(QTreeWidgetItem* item, int column)
{
    LinkTreeItem* linkTreeItem = dynamic_cast<LinkTreeItem*>(item);
    if(linkTreeItem){
        impl->sigItemChanged(linkTreeItem, column);
    }
}


SignalProxy<void()> LinkTreeWidget::sigSelectionChanged()
{
    return impl->sigSelectionChanged;
}


SignalProxy<void()> LinkTreeWidget::sigSelectionChanged(BodyItem* bodyItem)
{
    return impl->sigSelectionChangedOf(bodyItem);
}


Signal<void()>& LinkTreeWidgetImpl::sigSelectionChangedOf(BodyItem* bodyItem)
{
    BodyItemInfoPtr info = getBodyItemInfo(bodyItem);
    if(info){
        return info->sigSelectionChanged;
    }
    return dummySigSelectionChanged;
}


int LinkTreeWidget::selectedLinkIndex() const
{
    return impl->selectedLinkIndex(impl->currentBodyItem);
}


int LinkTreeWidget::selectedLinkIndex(BodyItem* bodyItem) const
{
    return impl->selectedLinkIndex(bodyItem);
}


int LinkTreeWidgetImpl::selectedLinkIndex(BodyItem* bodyItem) const
{
    BodyItemInfoPtr info = const_cast<LinkTreeWidgetImpl*>(this)->getBodyItemInfo(bodyItem);
    if(info){
        const auto& selection = info->selection;
        for(size_t i=0; i < selection.size(); ++i){
            if(selection[i]){
                return i;
            }
        }
    }
    return -1;
}


const std::vector<int>& LinkTreeWidget::selectedLinkIndices()
{
    return impl->selectedLinkIndices(impl->currentBodyItem);
}


const std::vector<int>& LinkTreeWidget::selectedLinkIndices(BodyItem* bodyItem)
{
    return impl->selectedLinkIndices(bodyItem);
}


const vector<int>& LinkTreeWidgetImpl::selectedLinkIndices(BodyItem* bodyItem)
{
    BodyItemInfoPtr info = getBodyItemInfo(bodyItem);
    if(info){
        info->selectedLinkIndices.clear();
        const auto& selection = info->selection;
        for(size_t i=0; i < selection.size(); ++i){
            if(selection[i]){
                info->selectedLinkIndices.push_back(i);
            }
        }
        return info->selectedLinkIndices;
    }
    return emptyLinkIndices;
}


const std::vector<bool>& LinkTreeWidget::linkSelection()
{
    return impl->linkSelection(impl->currentBodyItem);
}


const std::vector<bool>& LinkTreeWidget::linkSelection(BodyItem* bodyItem)
{
    return impl->linkSelection(bodyItem);
}


const vector<bool>& LinkTreeWidgetImpl::linkSelection(BodyItem* bodyItem)
{
    BodyItemInfoPtr info = getBodyItemInfo(bodyItem);
    if(info){
        return info->selection;
    }
    return emptySelection;
}


void LinkTreeWidget::onCustomContextMenuRequested(const QPoint& pos)
{
    impl->onCustomContextMenuRequested(pos);
}


void LinkTreeWidgetImpl::onCustomContextMenuRequested(const QPoint& pos)
{
    popupMenu.popup(pos);
}


void LinkTreeWidgetImpl::setExpansionState(const LinkTreeItem* item, bool on)
{
    if(listingMode == LinkTreeWidget::LINK_TREE || listingMode == LinkTreeWidget::JOINT_TREE){
        if(item->link()){
            currentBodyItemInfo->linkExpansions[item->link()->index()] = on;
        }

    } else if(listingMode == LinkTreeWidget::PART_TREE){
        if(on){
            currentBodyItemInfo->expandedParts.insert(item->name());
        } else {
            currentBodyItemInfo->expandedParts.erase(item->name());
        }
    }
}


void LinkTreeWidget::onItemExpanded(QTreeWidgetItem* treeWidgetItem)
{
    impl->onItemExpanded(treeWidgetItem);
}


void LinkTreeWidgetImpl::onItemExpanded(QTreeWidgetItem* treeWidgetItem)
{
    if(TRACE_FUNCTIONS){
        cout << "LinkTreeWidgetImpl::onItemExpanded()" << endl;
    }
    LinkTreeItem* item = dynamic_cast<LinkTreeItem*>(treeWidgetItem);
    if(item){
        setExpansionState(item, true);
        restoreSubTreeState(item);
    }
}


void LinkTreeWidget::onItemCollapsed(QTreeWidgetItem* treeWidgetItem)
{
    impl->onItemCollapsed(treeWidgetItem);
}


void LinkTreeWidgetImpl::onItemCollapsed(QTreeWidgetItem* treeWidgetItem)
{
    if(TRACE_FUNCTIONS){
        cout << "LinkTreeWidgetImpl::onItemCollapsed()" << endl;
    }
    LinkTreeItem* item = dynamic_cast<LinkTreeItem*>(treeWidgetItem);
    if(item){
        setExpansionState(item, false);
    }
}


void LinkTreeWidget::enableArchiveOfCurrentBodyItem(bool on)
{
    impl->isArchiveOfCurrentBodyItemEnabled = on;
}


bool LinkTreeWidget::makeSingleSelection(BodyItem* bodyItem, int linkIndex)
{
    return impl->makeSingleSelection(bodyItem, linkIndex);
}


bool LinkTreeWidgetImpl::makeSingleSelection(BodyItem* bodyItem, int linkIndex)
{
    BodyItemInfoPtr info = getBodyItemInfo(bodyItem);

    if(!info){
        return false;
    }

    auto& selection = info->selection;
    if(static_cast<size_t>(linkIndex) < selection.size()){
        if(!selection[linkIndex] || std::count(selection.begin(), selection.end(), true) > 1){
            std::fill(selection.begin(), selection.end(), false);
            selection[linkIndex] = true;
            
            if(bodyItem == currentBodyItem){
                restoreTreeState();

                LinkTreeItem* item = linkIndexToItemMap[linkIndex];
                if(item){
                    self->scrollToItem(item);
                }
                
                currentBodyItemInfo->sigSelectionChanged();
                sigSelectionChanged();
            } else {
                info->sigSelectionChanged();
            }
        }
    }
    return true;
}


MenuManager& LinkTreeWidget::popupMenuManager()
{
    return impl->popupMenuManager;
}


bool LinkTreeWidget::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool LinkTreeWidgetImpl::storeState(Archive& archive)
{
    if(listingModeCombo.isEnabled()){
        archive.write("listingMode", listingModeCombo.currentOrgText().toStdString(), DOUBLE_QUOTED);
    }

    if(isArchiveOfCurrentBodyItemEnabled){
        archive.writeItemId("currentBodyItem", currentBodyItem);
    }

    if(isCacheEnabled && !bodyItemInfoCache.empty()){

        ListingPtr bodyItemNodes = new Listing();

        for(BodyItemInfoMap::iterator it = bodyItemInfoCache.begin(); it != bodyItemInfoCache.end(); ++it){

            BodyItem* bodyItem = it->first;
            ValueNodePtr id = archive.getItemId(bodyItem);
            if(!id){ // The item is not in the item tree
                continue;
            }
            BodyItemInfo& info = *it->second;
            MappingPtr bodyItemNode = new Mapping();
            bool isEmpty = true;

            bodyItemNode->insert("id", id);
            
            const vector<int>& indices = selectedLinkIndices(bodyItem);
            if(!indices.empty()){
                Listing& selected = *bodyItemNode->createFlowStyleListing("selectedLinks");
                int n = indices.size();
                for(int i=0; i < n; ++i){
                    selected.append(indices[i], 20, n);
                }
                isEmpty = false;
            }

            auto exps = info.linkExpansions;
            int n = exps.size();
            int m = n - std::count(exps.begin(), exps.end(), true);
            if(m > 0){
                Listing& nonExpanded = *bodyItemNode->createFlowStyleListing("nonExpandedLinks");
                for(int i=0; i < n; ++i){
                    if(!exps[i]){
                        nonExpanded.append(i, 20, m);
                    }
                }
                isEmpty = false;
            }

            n = info.expandedParts.size();
            if(n > 0){
                Listing& expanded = *bodyItemNode->createFlowStyleListing("expandedParts");
                for(std::set<string>::iterator p = info.expandedParts.begin(); p != info.expandedParts.end(); ++p){
                    expanded.append(*p, 10, n, DOUBLE_QUOTED);
                }
                isEmpty = false;
            }

            if(!isEmpty){
                bodyItemNodes->append(bodyItemNode);
            }
        }

        if(!bodyItemNodes->empty()){
            archive.insert("bodyItems", bodyItemNodes);
        }
    }

    return true;
}


bool LinkTreeWidget::restoreState(const Archive& archive)
{
    archive.addPostProcess([&](){ impl->restoreState(archive); });
    return true;
}


bool LinkTreeWidgetImpl::restoreState(const Archive& archive)
{
    if(isCacheEnabled){
        const Listing& nodes = *archive.findListing("bodyItems");
        if(nodes.isValid() && !nodes.empty()){
            for(int i=0; i < nodes.size(); ++i){
                const Mapping& node = *nodes[i].toMapping();
                ValueNode* id = node.find("id");
                if(id->isValid()){
                    BodyItem* bodyItem = archive.findItem<BodyItem>(id);
                    if(bodyItem){
                        int numLinks = bodyItem->body()->numLinks();
                        BodyItemInfoPtr info = getBodyItemInfo(bodyItem);
                        const Listing& selected = *node.findListing("selectedLinks");
                        if(selected.isValid()){
                            info->setNumLinks(numLinks, true);
                            for(int i=0; i < selected.size(); ++i){
                                int index = selected[i].toInt();
                                if(index < numLinks){
                                    info->selection[index] = true;
                                }
                            }
                        }
                        const Listing& expanded = *node.findListing("expandedParts");
                        for(int i=0; i < expanded.size(); ++i){
                            info->expandedParts.insert(expanded[i]);
                        }
                    }
                }
            }
        }
    }

    bool updateListingMode = false;
    string listingModeString;
    if(archive.read("listingMode", listingModeString)){

        if(listingModeString != listingModeCombo.currentOrgText().toStdString()){
            listingModeCombo.blockSignals(true);
            if(listingModeCombo.findOrgText(listingModeString, true) >= 0){
                updateListingMode = true;
            }
            listingModeCombo.blockSignals(false);
        }
    }

    if(isArchiveOfCurrentBodyItemEnabled){
        setCurrentBodyItem(archive.findItem<BodyItem>("currentBodyItem"), updateListingMode);
    } else if(updateListingMode){
        setCurrentBodyItem(currentBodyItem, true);
    } else {
        restoreTreeState();
    }

    return true;
}
