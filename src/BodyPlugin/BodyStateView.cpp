/**
   @author Shin'ichiro Nakaoka
*/

#include "BodyStateView.h"
#include "BodyItem.h"
#include "BodySelectionManager.h"
#include <cnoid/TreeWidget>
#include <cnoid/ConnectionSet>
#include <cnoid/ExtraBodyStateAccessor>
#include <cnoid/BasicSensors>
#include <cnoid/EigenUtil>
#include <cnoid/ViewManager>
#include <QBoxLayout>
#include <QHeaderView>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;

struct DeviceTypeOrder
{
    bool operator()(const Device* lhs, const Device* rhs) const {
        int result = strcmp(typeid(*lhs).name(), typeid(*rhs).name());
        return (result == 0) ? (lhs->id() <= rhs->id()) : (result <= 0);
    }
};

}

namespace cnoid {

class StateItem : public QTreeWidgetItem
{
public:
    StateItem() { lastNumValidColumns = 0; }
    int lastNumValidColumns;
};

class BodyStateView::Impl
{
public:
    BodyStateView* self;

    TreeWidget stateTreeWidget;

    BodyItemPtr currentBodyItem;
    BodyPtr currentBody;

    vector<double> buf;

    PolymorphicReferencedArray<ExtraBodyStateAccessor> accessors;
    vector< vector<int> > extraStateItemMap;
    vector<ExtraBodyStateAccessor::Value> extraState;

    Connection bodyItemChangeConnection;
    ConnectionSet stateConnections;

    Impl(BodyStateView* self);
    ~Impl();
    void clearSignalConnections();
    void onActivated(bool on);
    void setCurrentBodyItem(BodyItem* bodyItem);
    void updateStateList(BodyItem* bodyItem);
    void updateDeviceStates(DevicePtr device, int rowIndex);
    void updateExtraStates();
};
}


void BodyStateView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<BodyStateView>(
        "BodyStateView", N_("Body State"), ViewManager::SINGLE_OPTIONAL);
}


BodyStateView::BodyStateView()
{
    impl = new Impl(this);
}


BodyStateView::Impl::Impl(BodyStateView* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::BOTTOM);
    
    QVBoxLayout* vbox = new QVBoxLayout();

    stateTreeWidget.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    stateTreeWidget.setSelectionMode(QAbstractItemView::NoSelection);
    stateTreeWidget.setAlternatingRowColors(true);
    stateTreeWidget.setVerticalGridLineShown(true);
    stateTreeWidget.setAllColumnsShowFocus(true);
    stateTreeWidget.setHeaderHidden(true);
    stateTreeWidget.setRootIsDecorated(false);
    QHeaderView* header = stateTreeWidget.header();
    header->setStretchLastSection(false);
    header->setMinimumSectionSize(0);
    
    stateTreeWidget.setColumnCount(1);

    vbox->addWidget(&stateTreeWidget);
    self->setLayout(vbox);

    self->sigActivated().connect([&](){ onActivated(true); });
    self->sigDeactivated().connect([&](){ onActivated(false); });

    //self->enableFontSizeZoomKeys(true);
}


BodyStateView::~BodyStateView()
{
    delete impl;
}


BodyStateView::Impl::~Impl()
{
    clearSignalConnections();
}


void BodyStateView::Impl::clearSignalConnections()
{
    bodyItemChangeConnection.disconnect();
    stateConnections.disconnect();
}    


void BodyStateView::Impl::onActivated(bool on)
{
    clearSignalConnections();

    if(!on){
        setCurrentBodyItem(0);

    } else {
        auto bsm = BodySelectionManager::instance();
        setCurrentBodyItem(bsm->currentBodyItem());
        bodyItemChangeConnection =
            bsm->sigCurrentBodyItemChanged().connect(
                [&](BodyItem* bodyItem){ setCurrentBodyItem(bodyItem); });
    }
}


void BodyStateView::Impl::setCurrentBodyItem(BodyItem* bodyItem)
{
    currentBodyItem = bodyItem;
    if(bodyItem){
        currentBody = bodyItem->body();
    } else {
        currentBody.reset();
    }
    
    updateStateList(bodyItem);
}


void BodyStateView::Impl::updateStateList(BodyItem* bodyItem)
{
    stateConnections.disconnect();
    stateTreeWidget.clear();
    int maxNumStateElements = 0;

    if(currentBody){

        DeviceList<> devices(currentBody->devices());
        DeviceList<> targetDevices;
        targetDevices << devices.extract<ForceSensor>();
        targetDevices << devices.extract<RateGyroSensor>();
        targetDevices << devices.extract<AccelerationSensor>();
        std::stable_sort(targetDevices.begin(), targetDevices.end(), DeviceTypeOrder());
            
        for(size_t i=0; i < targetDevices.size(); ++i){
            Device* device = targetDevices[i];
            StateItem* deviceItem = new StateItem();
        
            deviceItem->setText(0, QString(" %1 ").arg(device->name().c_str()));
            stateTreeWidget.addTopLevelItem(deviceItem);

            if(device->stateSize() > maxNumStateElements){
                maxNumStateElements = device->stateSize();
            }
            stateConnections.add(
                device->sigStateChanged().connect(
                    [=](){ updateDeviceStates(device, i); }));
            updateDeviceStates(device, i);
        }

        extraStateItemMap.clear();
        vector<string> names;
        currentBody->getCaches(accessors, names);
        for(size_t i=0; i < accessors.size(); ++i){
            ExtraBodyStateAccessor& accessor = *accessors[i];
            vector<int> itemMap;
            accessor.getState(extraState);
            if(!extraState.empty()){
                for(size_t j=0; j < extraState.size(); ++j){
                    QTreeWidgetItem* stateItem = new StateItem();
                    stateItem->setText(0, QString(" %1 ").arg(accessor.getStateItemLabel(j)));
                    itemMap.push_back(stateTreeWidget.topLevelItemCount());
                    stateTreeWidget.addTopLevelItem(stateItem);
                    int m = ExtraBodyStateAccessor::getNumValueElements(extraState[j]);
                    if(m > maxNumStateElements){
                        maxNumStateElements = m;
                    }
                }
                stateConnections.add(
                    accessor.sigStateChanged().connect(
                        [&](){ updateExtraStates(); }));
            }
            extraStateItemMap.push_back(itemMap);
        }
        if(!extraStateItemMap.empty()){
            updateExtraStates();
        }
    }
    
    int lastColumn = maxNumStateElements + 1;
    stateTreeWidget.setColumnCount(lastColumn + 1);
    stateTreeWidget.setHeaderSectionResizeMode(0, QHeaderView::ResizeToContents);
    for(int i=0; i < maxNumStateElements; ++i){
        stateTreeWidget.setHeaderSectionResizeMode(i+1, QHeaderView::Stretch);
    }
    stateTreeWidget.setHeaderSectionResizeMode(lastColumn, QHeaderView::Fixed);
    stateTreeWidget.header()->resizeSection(lastColumn, 0);

    const int n = stateTreeWidget.topLevelItemCount();
    for(int i=0; i < n; ++i){
        QTreeWidgetItem* item = stateTreeWidget.topLevelItem(i);
        item->setTextAlignment(0, Qt::AlignHCenter);
        for(int j=0; j < maxNumStateElements; ++j){
            //item->setTextAlignment(j + 1, Qt::AlignHCenter);
            item->setTextAlignment(j + 1, Qt::AlignRight);
        }
    }
}


void BodyStateView::Impl::updateDeviceStates(DevicePtr device, int rowIndex)
{
    if(currentBody){
        StateItem* item = static_cast<StateItem*>(stateTreeWidget.topLevelItem(rowIndex));
        int size = device->stateSize();
        buf.resize(size);
        device->writeState(&buf.front());
        int i;
        for(i=0; i < size; ++i){
            item->setText(i + 1, QString::number(buf[i], 'f', 2));
        }
        for(   ; i < item->lastNumValidColumns; ++i){
            item->setText(i + 1, "");
        }
        item->lastNumValidColumns = size;
    }
}


void BodyStateView::Impl::updateExtraStates()
{
    if(!currentBody){
        return;
    }
    const int prec = 3;
    
    for(size_t i=0 ;i < accessors.size(); ++i){
        const ExtraBodyStateAccessor& accessor = *accessors[i];
        const vector<int>& itemMap = extraStateItemMap[i];
        accessor.getState(extraState);
        for(size_t j=0; j < extraState.size(); ++j){
            const ExtraBodyStateAccessor::Value& s = extraState[j];
            StateItem* item = static_cast<StateItem*>(stateTreeWidget.topLevelItem(itemMap[j]));
            int column = 1;
            switch(s.which()){
            case ExtraBodyStateAccessor::BOOL:
                item->setText(column++, s.getBool() ? _("ON") : _("OFF"));
                break;
            case ExtraBodyStateAccessor::INT:
                item->setText(column++, QString::number(s.getInt(), 'f', prec));
                break;
            case ExtraBodyStateAccessor::DOUBLE:
                item->setText(column++, QString::number(s.getDouble(), 'f', prec));
                break;
            case ExtraBodyStateAccessor::ANGLE:
                item->setText(column++, QString::number(s.getAngle(), 'f', prec));
                break;
            case ExtraBodyStateAccessor::STRING:
                item->setText(column++, s.getString().c_str());
                break;
                /*
                  case ExtraBodyStateAccessor::VECTOR2:
                  {
                  const Vector2& v = get<Vector2>(s);
                  item->setText(column++, QString::number(v[0], 'f', prec));
                  item->setText(column++, QString::number(v[1], 'f', prec));
                  break;
                  }
                */
            case ExtraBodyStateAccessor::VECTOR3:
            {
                const Vector3f& v = s.getVector3f();
                item->setText(column++, QString::number(v[0], 'f', prec));
                item->setText(column++, QString::number(v[1], 'f', prec));
                item->setText(column++, QString::number(v[2], 'f', prec));
                break;
            }
            case ExtraBodyStateAccessor::VECTORX:
            {
                const VectorX& v = s.getVectorX();
                for(int i=0; i < v.size(); ++i){
                    item->setText(column++, QString::number(v[i], 'f', prec));
                }
                break;
            }
            default:
                break;
            }
            for(int i=column; i < item->lastNumValidColumns; ++i){
                item->setText(i, "");
            }
            item->lastNumValidColumns = column;
        }
    }
}


bool BodyStateView::storeState(Archive& archive)
{
    return true;
}


bool BodyStateView::restoreState(const Archive& archive)
{
    return true;
}
