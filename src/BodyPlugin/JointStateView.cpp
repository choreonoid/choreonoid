/**
   @author Shin'ichiro Nakaoka
*/

#include "JointStateView.h"
#include "LinkTreeWidget.h"
#include "BodySelectionManager.h"
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

    LinkTreeWidget jointStateWidget;
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
};

}


void JointStateView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<JointStateView>(
        "JointStateView", N_("Joint State"), ViewManager::SINGLE_OPTIONAL);
}


JointStateView::JointStateView()
{
    impl = new Impl(this);
}


JointStateView::Impl::Impl(JointStateView* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::CENTER);
    
    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->setSpacing(0);

    //vbox->addWidget(jointStateWidget.listingModeCombo());

    jointStateWidget.setNameColumnMarginEnabled(true);
    //jointStateWidget.setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    jointStateWidget.setSelectionMode(QAbstractItemView::NoSelection);
    jointStateWidget.setAlternatingRowColors(true);
    jointStateWidget.setVerticalGridLineShown(true);
    jointStateWidget.setAllColumnsShowFocus(true);
    jointStateWidget.enableCache(true);
    jointStateWidget.enableArchiveOfCurrentBodyItem(true);
    jointStateWidget.setListingMode(LinkTreeWidget::JOINT_LIST);

    QHeaderView* header = jointStateWidget.header();
    header->setDefaultAlignment(Qt::AlignHCenter);

    qColumn = jointStateWidget.addColumn(_("Angle"));
    uColumn = jointStateWidget.addColumn(_("Torque"));

    QTreeWidgetItem* headerItem = jointStateWidget.headerItem();
    headerItem->setTextAlignment(qColumn, Qt::AlignRight);
    headerItem->setTextAlignment(uColumn, Qt::AlignRight);

    header->setMinimumSectionSize(0);
    jointStateWidget.setHeaderSectionResizeMode(jointStateWidget.nameColumn(), QHeaderView::ResizeToContents);
    jointStateWidget.setHeaderSectionResizeMode(jointStateWidget.jointIdColumn(), QHeaderView::ResizeToContents);
    if(doColumnStretch){
        jointStateWidget.setHeaderSectionResizeMode(qColumn, QHeaderView::Stretch);
        jointStateWidget.setHeaderSectionResizeMode(uColumn, QHeaderView::Stretch);
    } else {
        jointStateWidget.setHeaderSectionResizeMode(qColumn, QHeaderView::ResizeToContents);
        jointStateWidget.setHeaderSectionResizeMode(uColumn, QHeaderView::ResizeToContents);
    }
        
    int lastColumn = jointStateWidget.addColumn();
    if(doColumnStretch){
        jointStateWidget.setHeaderSectionResizeMode(lastColumn, QHeaderView::Fixed);
        header->resizeSection(lastColumn, 0);
    } else {
        jointStateWidget.setHeaderSectionResizeMode(lastColumn, QHeaderView::Stretch);
    }
    
    jointStateWidget.sigUpdateRequest().connect([&](bool){ updateJointList(); });
    
    vbox->addWidget(&jointStateWidget);

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
    jointStateWidget.setNumColumns(uColumn + 1);
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
                        columnMap.push_back(jointStateWidget.addColumn(accessor.getJointStateItemLabel(j)));
                    }
                }
                jointStateColumnMap.push_back(columnMap);
            }
        }
        const int m = jointStateWidget.columnCount();
        for(int i = qColumn; i < m; ++i){
            if(doColumnStretch){
                jointStateWidget.setHeaderSectionResizeMode(i, QHeaderView::Stretch);
            } else {
                jointStateWidget.setHeaderSectionResizeMode(i, QHeaderView::ResizeToContents);
            }
        }
    }
    
    const int lastColumn = jointStateWidget.addColumn();
    if(doColumnStretch){
        jointStateWidget.setHeaderSectionResizeMode(lastColumn, QHeaderView::Fixed);
        jointStateWidget.header()->resizeSection(lastColumn, 0);
    } else {
        jointStateWidget.setHeaderSectionResizeMode(lastColumn, QHeaderView::Stretch);
    }

    jointStateWidget.setBodyItem(bodyItem);

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
    // The current body item should be gotten from the jointStateWidget because
    // this function is first called from it when the body item is detached.
    BodyItem* bodyItem = jointStateWidget.bodyItem();
    if(bodyItem){

        QTreeWidgetItem* headerItem = jointStateWidget.headerItem();        
        const int n = jointStateWidget.columnCount();
        for(int i = uColumn + 1; i < n; ++i){
            headerItem->setTextAlignment(i, Qt::AlignRight);
        }

        BodyPtr body = bodyItem->body();
        int nameColumn = jointStateWidget.nameColumn();
        for(int i = 0; i < currentBody->numJoints(); ++i){
            Link* joint = currentBody->joint(i);
            if(joint){
                LinkTreeItem* item = jointStateWidget.itemOfLink(joint->index());
                item->setTextAlignment(nameColumn, Qt::AlignHCenter);
                item->setTextAlignment(qColumn, Qt::AlignRight);
                item->setTextAlignment(uColumn, Qt::AlignRight);
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
                            LinkTreeItem* item = jointStateWidget.itemOfLink(joint->index());
                            item->setTextAlignment(column, alignment);
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
                LinkTreeItem* item = jointStateWidget.itemOfLink(joint->index());
                if(joint->jointType() == Link::ROTATIONAL_JOINT){
                    item->setText(qColumn, QString::number(degree(joint->q()), 'f', 2));
                } else {
                    item->setText(qColumn, QString::number(joint->q(), 'f', 2));
                }
                item->setText(uColumn, QString::number(joint->u(), 'f', 2));
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
            const int m = jointState.colSize();
            for(int j=0; j < n; ++j){
                Link* joint = currentBody->joint(j);
                Array2D<ExtraBodyStateAccessor::Value>::Row js = jointState.row(j);
                if(joint){
                    LinkTreeItem* item = jointStateWidget.itemOfLink(joint->index());
                    for(int k=0; k < m; ++k){
                        bool isValid = true;
                        const int column = columnMap[k];
                        const ExtraBodyStateAccessor::Value& v = js[k];
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
            }
        }
        isExtraJointStateChanged = false;
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
