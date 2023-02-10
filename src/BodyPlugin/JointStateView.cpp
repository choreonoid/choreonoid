#include "JointStateView.h"
#include "LinkDeviceTreeWidget.h"
#include "BodySelectionManager.h"
#include <cnoid/BodyItem>
#include <cnoid/Body>
#include <cnoid/ConnectionSet>
#include <cnoid/EigenUtil>
#include <cnoid/ExtraBodyStateAccessor>
#include <cnoid/LazyCaller>
#include <cnoid/ViewManager>
#include <QBoxLayout>
#include <QHeaderView>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;
const bool doColumnStretch = true;

}

namespace cnoid {

class JointStateView::Impl
{
public:
    JointStateView* self;

    LinkDeviceTreeWidget treeWidget;
    int qColumn;
    int uColumn;

    BodyPtr currentBody;
    PolymorphicReferencedArray<ExtraBodyStateAccessor> accessors;
    vector< vector<int> > jointStateColumnMap;
    Array2D<ExtraBodyStateAccessor::Value> jointState;

    bool isKinematicStateChanged;
    bool isExtraJointStateChanged;
    LazyCaller updateViewLater;
    ConnectionSet connections;
    ConnectionSet connectionsToBody;

    Impl(JointStateView* self);
    ~Impl();
    void clearSignalConnections();
    void onActivated(bool on);
    void setCurrentBodyItem(BodyItem* bodyItem);
    void updateJointList();
    void onKinematicStateChanged();
    void onExtraJointStateChanged();
    void updateView();
    void updateJointItem(int index, LinkDeviceTreeItem* item, const vector<int>& columnMap);
};

}


void JointStateView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<JointStateView>(N_("JointStateView"), N_("Joint State"));
}


JointStateView::JointStateView()
{
    impl = new Impl(this);
}


JointStateView::Impl::Impl(JointStateView* self)
    : self(self)
{
    self->setDefaultLayoutArea(BottomCenterArea);
    
    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->setSpacing(0);

    treeWidget.setLinkItemVisible(false);
    treeWidget.setJointItemVisible(true);
    treeWidget.setNumberColumnMode(LinkDeviceTreeWidget::Identifier);
    treeWidget.setSelectionMode(QAbstractItemView::NoSelection);
    treeWidget.setAlternatingRowColors(true);
    treeWidget.setVerticalGridLineShown(true);
    treeWidget.setAllColumnsShowFocus(true);

    QHeaderView* header = treeWidget.header();
    header->setDefaultAlignment(Qt::AlignHCenter);

    qColumn = treeWidget.addColumn(_("Displacement"));
    uColumn = treeWidget.addColumn(_("Torque"));

    QTreeWidgetItem* headerItem = treeWidget.headerItem();
    headerItem->setTextAlignment(qColumn, Qt::AlignRight);
    headerItem->setTextAlignment(uColumn, Qt::AlignRight);

    treeWidget.setHeaderSectionResizeMode(treeWidget.nameColumn(), QHeaderView::ResizeToContents);
    treeWidget.setHeaderSectionResizeMode(treeWidget.numberColumn(), QHeaderView::ResizeToContents);
    if(doColumnStretch){
        treeWidget.setHeaderSectionResizeMode(qColumn, QHeaderView::Stretch);
        treeWidget.setHeaderSectionResizeMode(uColumn, QHeaderView::Stretch);
    } else {
        treeWidget.setHeaderSectionResizeMode(qColumn, QHeaderView::ResizeToContents);
        treeWidget.setHeaderSectionResizeMode(uColumn, QHeaderView::ResizeToContents);
    }
        
    int lastColumn = treeWidget.addColumn();
    if(doColumnStretch){
        treeWidget.setHeaderSectionResizeMode(lastColumn, QHeaderView::Fixed);
        header->resizeSection(lastColumn, 0);
    } else {
        treeWidget.setHeaderSectionResizeMode(lastColumn, QHeaderView::Stretch);
    }
    
    treeWidget.sigUpdateRequest().connect([&](bool){ updateJointList(); });
    
    vbox->addWidget(&treeWidget);

    self->setLayout(vbox);

    self->sigActivated().connect([&](){ onActivated(true); });
    self->sigDeactivated().connect([&](){ onActivated(false); });

    updateViewLater.setFunction([&](){ updateView(); });

    //self->enableFontSizeZoomKeys(true);
}


JointStateView::~JointStateView()
{
    delete impl;
}


JointStateView::Impl::~Impl()
{
    clearSignalConnections();
}


void JointStateView::Impl::clearSignalConnections()
{
    connections.disconnect();
    connectionsToBody.disconnect();
}    


void JointStateView::Impl::onActivated(bool on)
{
    clearSignalConnections();

    if(!on){
        setCurrentBodyItem(0);

    } else {
        auto bsm = BodySelectionManager::instance();
        connections.add(
            bsm->sigCurrentBodyItemChanged().connect(
                [&](BodyItem* bodyItem){ setCurrentBodyItem(bodyItem); }));
        setCurrentBodyItem(bsm->currentBodyItem());
    }
}


void JointStateView::Impl::setCurrentBodyItem(BodyItem* bodyItem)
{
    connectionsToBody.disconnect();
    treeWidget.setNumColumns(uColumn + 1);
    jointStateColumnMap.clear();

    if(!bodyItem){
        currentBody.reset();
        accessors.clear();
        
    } else {
        currentBody = bodyItem->body();

        vector<string> names;
        currentBody->getCaches(accessors, names);
        vector<int> columnMap;
        for(size_t i=0; i < accessors.size(); ++i){
            ExtraBodyStateAccessor& accessor = *accessors[i];
            const int n = accessor.getNumJointStateItems();
            if(n > 0){
                columnMap.clear();
                for(int j=0; j < n; ++j){
                    const char* itemName = accessor.getJointStateItemName(j);
                    if(strcmp(itemName, "Pos") == 0){
                        columnMap.push_back(qColumn);
                    } else {
                        columnMap.push_back(treeWidget.addColumn(accessor.getJointStateItemLabel(j)));
                    }
                }
                jointStateColumnMap.push_back(columnMap);
            }
        }
        const int m = treeWidget.columnCount();
        for(int i = qColumn; i < m; ++i){
            if(doColumnStretch){
                treeWidget.setHeaderSectionResizeMode(i, QHeaderView::Stretch);
            } else {
                treeWidget.setHeaderSectionResizeMode(i, QHeaderView::ResizeToContents);
            }
        }
    }
    
    const int lastColumn = treeWidget.addColumn();
    if(doColumnStretch){
        treeWidget.setHeaderSectionResizeMode(lastColumn, QHeaderView::Fixed);
        treeWidget.header()->resizeSection(lastColumn, 0);
    } else {
        treeWidget.setHeaderSectionResizeMode(lastColumn, QHeaderView::Stretch);
    }

    treeWidget.setBodyItem(bodyItem);

    if(bodyItem){
        connectionsToBody.add(
            bodyItem->sigKinematicStateChanged().connect(
                [&](){ onKinematicStateChanged(); }));
    }

    for(size_t i=0; i < accessors.size(); ++i){
        connectionsToBody.add(
            accessors[i]->sigStateChanged().connect(
                [&](){ onExtraJointStateChanged(); }));
    }
}


void JointStateView::Impl::updateJointList()
{
    // The current body item should be gotten from the treeWidget because
    // this function is first called from it when the body item is detached.
    BodyItem* bodyItem = treeWidget.bodyItem();
    if(bodyItem){

        QTreeWidgetItem* headerItem = treeWidget.headerItem();        
        const int n = treeWidget.columnCount();
        for(int i = uColumn + 1; i < n; ++i){
            headerItem->setTextAlignment(i, Qt::AlignRight);
        }

        BodyPtr body = bodyItem->body();
        int nameColumn = treeWidget.nameColumn();
        for(int i = 0; i < currentBody->numJoints(); ++i){
            Link* joint = currentBody->joint(i);
            if(joint){
                if(auto item = treeWidget.itemOfLink(joint->index())){
                    item->setTextAlignment(nameColumn, Qt::AlignHCenter);
                    item->setTextAlignment(qColumn, Qt::AlignRight);
                    item->setTextAlignment(uColumn, Qt::AlignRight);
                }
            }
        }

        for(size_t i=0; i < accessors.size(); ++i){
            ExtraBodyStateAccessor& accessor = *accessors[i];
            const vector<int>& columnMap = jointStateColumnMap[i];
            jointState.clear();
            accessor.getJointState(jointState);
            const int nc = jointState.colSize();
            const int nj = jointState.rowSize();
            if(nj > 0){
                for(int j=0; j < nc; ++j){
                    const int column = columnMap[j];
                    Array2D<ExtraBodyStateAccessor::Value>::Column js = jointState.column(j);
                    const ExtraBodyStateAccessor::Value& v = js[0];
                    Qt::Alignment alignment = Qt::AlignHCenter;
                    int type = v.which();
                    if(type == ExtraBodyStateAccessor::INT ||
                       type == ExtraBodyStateAccessor::DOUBLE ||
                       type == ExtraBodyStateAccessor::ANGLE){
                        alignment = Qt::AlignRight;
                    }
                    headerItem->setTextAlignment(column, alignment);

                    for(int k=0; k < nj; ++k){
                        Link* joint = currentBody->joint(k);
                        if(joint){
                            if(auto item = treeWidget.itemOfLink(joint->index())){
                                item->setTextAlignment(column, alignment);
                            }
                        }
                    }
                }
            }
        }

        isKinematicStateChanged = true;
        isExtraJointStateChanged = true;
        updateView();
    }
}


void JointStateView::Impl::onKinematicStateChanged()
{
    isKinematicStateChanged = true;
    updateViewLater();
}


void JointStateView::Impl::onExtraJointStateChanged()
{
    isExtraJointStateChanged = true;
    updateViewLater();
}


void JointStateView::Impl::updateView()
{
    if(!currentBody){
        return;
    }

    if(isKinematicStateChanged){
        for(int i = 0; i < currentBody->numJoints(); ++i){
            Link* joint = currentBody->joint(i);
            if(joint){
                if(auto item = treeWidget.itemOfLink(joint->index())){
                    if(joint->jointType() == Link::RevoluteJoint){
                        item->setText(qColumn, QString::number(degree(joint->q()), 'f', 2));
                    } else {
                        item->setText(qColumn, QString::number(joint->q(), 'f', 2));
                    }
                    item->setText(uColumn, QString::number(joint->u(), 'f', 2));
                }
            }
        }
        isKinematicStateChanged = false;
    }

    if(isExtraJointStateChanged){
        for(size_t i=0; i < accessors.size(); ++i){
            ExtraBodyStateAccessor& accessor = *accessors[i];
            const vector<int>& columnMap = jointStateColumnMap[i];
            jointState.clear();
            accessor.getJointState(jointState);
            const int n = jointState.rowSize();
            for(int j=0; j < n; ++j){
                if(auto joint = currentBody->joint(j)){
                    if(auto item = treeWidget.itemOfLink(joint->index())){
                        updateJointItem(j, item, columnMap);
                    }
                }
            }
        }
        isExtraJointStateChanged = false;
    }
}


void JointStateView::Impl::updateJointItem(int index, LinkDeviceTreeItem* item, const vector<int>& columnMap)
{
    auto js = jointState.row(index);
    const int n = jointState.colSize();
    for(int i=0; i < n; ++i){
        bool isValid = true;
        const int column = columnMap[i];
        const ExtraBodyStateAccessor::Value& v = js[i];
        switch(v.which()){
        case ExtraBodyStateAccessor::BOOL:
            item->setText(column, v.getBool() ? _("ON") : _("OFF"));
            break;
        case ExtraBodyStateAccessor::INT:
            item->setText(column, QString::number(v.getInt()));
            break;
        case ExtraBodyStateAccessor::DOUBLE:
            item->setText(column, QString::number(v.getDouble()));
            break;
        case ExtraBodyStateAccessor::ANGLE:
            item->setText(column, QString::number(degree(v.getAngle()), 'f', 1));
            break;
        case ExtraBodyStateAccessor::STRING:
            item->setText(column, v.getString().c_str());
            break;
            //case ExtraBodyStateAccessor::VECTOR2:
        case ExtraBodyStateAccessor::VECTOR3:
        case ExtraBodyStateAccessor::VECTORX:
            item->setText(column, "...");
            break;
        default:
            item->setText(column, "");
            isValid = false;
            break;
        }
        if(isValid){
            const int attr = v.attribute();
            if(attr == ExtraBodyStateAccessor::NORMAL){
                item->setData(column, Qt::ForegroundRole, QVariant());
            } else if(attr | ExtraBodyStateAccessor::WARNING){
                item->setData(column, Qt::ForegroundRole, QBrush(Qt::red));
            }
        }
    }
}


bool JointStateView::storeState(Archive& archive)
{
    return true;
}


bool JointStateView::restoreState(const Archive& archive)
{
    return true;
}
