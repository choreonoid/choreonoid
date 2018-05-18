/**
   @author Shizuko Hattori
*/

#include "OldRTMPointCloudIOItem.h"
#include "corba/PointCloud.hh"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/Config>
#include <cnoid/RangeSensor>
#include <cnoid/RangeCamera>
#include <cnoid/MessageView>
#include <cnoid/OpenRTMUtil>
#include <cnoid/PointSetItem>
#include <cnoid/LazyCaller>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/InterfaceDataTypes.hh>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace RTC;

namespace {

class PointCloudInPort
{
public:
    InPort<PointCloudTypes::PointCloud> inPort;
    PointCloudTypes::PointCloud timedPointCloud;
    string portName;
    RangeCameraPtr rangeCamera;

    PointCloudInPort(RangeCamera* rangeCamera, const string& name)
        : inPort(name.c_str(), timedPointCloud),
          portName(name),
          rangeCamera(rangeCamera) { }
};
typedef std::shared_ptr<PointCloudInPort> PointCloudInPortPtr;


class PointCloudIORTC : public DataFlowComponentBase {
public:

    typedef map<string, PointCloudInPortPtr> PointCloudInPortMap;
    PointCloudInPortMap pointCloudInPorts;

    struct PointCloudField {
        unsigned long offset;
        PointCloudTypes::DataType type;
        unsigned long count;
    };
    std::map<string, PointCloudField> pointCloudField;

    static void registerFactory(RTC::Manager* manager, const char* componentTypeName);

    PointCloudIORTC(RTC::Manager* manager);
    ~PointCloudIORTC(){ };

    bool createPort(DeviceList<RangeCamera> rangeCameras, vector<string> pointCloudPortNames);
    float readFloatPointCloudData(unsigned char* src, PointCloudField& fe, bool is_bigendian);

    virtual ReturnCode_t onInitialize();
    virtual ReturnCode_t onActivated(UniqueId ec_id);
    virtual ReturnCode_t onDeactivated(UniqueId ec_id);
    virtual ReturnCode_t onExecute(UniqueId ec_id);
};


void PointCloudIORTC::registerFactory(RTC::Manager* manager, const char* componentTypeName)
{
  static const char* pointCloudIORTC_spec[] = {
    "implementation_id", "OldPointCloudIO",
    "type_name",         "OldPointCloudIO",
    "description",       "This component is the pointCloud visualization component.",
    "version",           CNOID_VERSION_STRING,
    "vendor",            "AIST",
    "category",          "choreonoid",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    "conf.default.debugLevel", "0",
    ""
  };

  RTC::Properties profile(pointCloudIORTC_spec);
  profile.setDefault("type_name", componentTypeName);
  manager->registerFactory(profile,
                           RTC::Create<PointCloudIORTC>,
                           RTC::Delete<PointCloudIORTC>);
}


PointCloudIORTC::PointCloudIORTC(Manager* manager)
    : DataFlowComponentBase(manager)
{
    pointCloudField.clear();
    pointCloudField["x"].offset = 0;
    pointCloudField["x"].type = PointCloudTypes::FLOAT32;
    pointCloudField["x"].count = 4;
    pointCloudField["y"].offset = 4;
    pointCloudField["y"].type = PointCloudTypes::FLOAT32;
    pointCloudField["y"].count = 4;
    pointCloudField["z"].offset = 8;
    pointCloudField["z"].type = PointCloudTypes::FLOAT32;
    pointCloudField["z"].count = 4;
    pointCloudField["r"].offset = 12;
    pointCloudField["r"].type = PointCloudTypes::UINT8;
    pointCloudField["r"].count = 1;
    pointCloudField["g"].offset = 13;
    pointCloudField["g"].type = PointCloudTypes::UINT8;
    pointCloudField["g"].count = 1;
    pointCloudField["b"].offset = 14;
    pointCloudField["b"].type = PointCloudTypes::UINT8;
    pointCloudField["b"].count = 1;
}


bool PointCloudIORTC::createPort(DeviceList<RangeCamera> rangeCameras, vector<string> pointCloudPortNames)
{
    bool ret = true;

    for(size_t i=0; i < rangeCameras.size(); i++){
        PointCloudInPort* pointCloudInPort = new PointCloudInPort(rangeCameras[i], pointCloudPortNames[i]);
        if(!addInPort(pointCloudInPort->portName.c_str(), pointCloudInPort->inPort)){
            ret = false;
        } else {
            pointCloudInPorts.insert(make_pair(pointCloudInPort->portName, PointCloudInPortPtr(pointCloudInPort)));
        }
    }

    return ret;
}


ReturnCode_t PointCloudIORTC::onInitialize()
{
    return RTC_OK;
}


ReturnCode_t PointCloudIORTC::onActivated(UniqueId ec_id)
{
     return RTC_OK;
}

ReturnCode_t PointCloudIORTC::onDeactivated(UniqueId ec_id)
{
    return RTC_OK;
}


float PointCloudIORTC::readFloatPointCloudData(unsigned char* src, PointCloudField& fe, bool is_bigendian)
{
    unsigned char* src_;
    unsigned char buf[8];
    if(is_bigendian){
        for(size_t i=0; i < fe.count; i++){
            buf[i] = src[fe.count-1-i];
        }
        src_ = buf;
    } else {
        src_ = src;
    }

    switch(fe.type){
    case PointCloudTypes::FLOAT32 :
        return *((float *)src_);

    case PointCloudTypes::FLOAT64 :
        return *((double *)src_);

    default:
        break;
    }
    return 0;
}


ReturnCode_t PointCloudIORTC::onExecute(UniqueId ec_id)
{
    for(PointCloudInPortMap::iterator it = pointCloudInPorts.begin();
            it != pointCloudInPorts.end(); it++){
        InPort<PointCloudTypes::PointCloud>& inport_ = it->second->inPort;
        if(inport_.isNew()){
            do {
                inport_.read();
            } while(inport_.isNew());

            PointCloudTypes::PointCloud& timedPointCloud = it->second->timedPointCloud;
            RangeCamera* rangeCamera = it->second->rangeCamera;

            int numPoints = timedPointCloud.height * timedPointCloud.width;
            bool rgb = false;
            for(size_t i=0; i < timedPointCloud.fields.length(); i++){
                string name = string(timedPointCloud.fields[i].name);
                pointCloudField[name].offset = timedPointCloud.fields[i].offset;
                pointCloudField[name].type = timedPointCloud.fields[i].data_type;
                pointCloudField[name].count = timedPointCloud.fields[i].count;
                if(i==5){
                    rgb = true;
                }
            }
            unsigned char* src = (unsigned char*)timedPointCloud.data.get_buffer();
            std::shared_ptr<RangeCamera::PointData> tmpPoints = std::make_shared< vector<Vector3f> >();
            std::shared_ptr<Image> tmpImage;
            unsigned char* pixels = 0;
            if(rgb){
                tmpImage = std::make_shared<Image>();
                tmpImage->setSize(timedPointCloud.width, timedPointCloud.height, 3);
                pixels = tmpImage->pixels();
            }

            tmpPoints->clear();
            tmpPoints->reserve(numPoints);

            PointCloudField& fex = pointCloudField["x"];
            PointCloudField& fey = pointCloudField["y"];
            PointCloudField& fez = pointCloudField["z"];
            PointCloudField& fer = pointCloudField["r"];
            PointCloudField& feg = pointCloudField["g"];
            PointCloudField& feb = pointCloudField["b"];

            for(int i=0; i < numPoints; i++, src+=timedPointCloud.point_step){
                Vector3f point;
                point.x() = readFloatPointCloudData(&src[fex.offset], fex, timedPointCloud.is_bigendian );
                point.y() = readFloatPointCloudData(&src[fey.offset], fey, timedPointCloud.is_bigendian );
                point.z() = readFloatPointCloudData(&src[fez.offset], fez, timedPointCloud.is_bigendian );
                tmpPoints->push_back(point);

                if(rgb && pixels){
                    pixels[0] = src[fer.offset];
                    pixels[1] = src[feg.offset];
                    pixels[2] = src[feb.offset];
                    pixels += 3;
                }
            }
            rangeCamera->setPoints(tmpPoints);
            if(rgb && pixels){
                rangeCamera->setImage(tmpImage);
            }

            callLater( [rangeCamera](){rangeCamera->notifyStateChange();} );
        }
    }

    return RTC::RTC_OK;
}

}

namespace cnoid {

class OldRTMPointCloudIOItemImpl
{
public:
    OldRTMPointCloudIOItem* self;
    MessageView* mv;
    OpenRTM::ExtTrigExecutionContextService_var execContext;
    bool isChoreonoidExecutionContext;

    Body* body;
    DeviceList<RangeCamera> rangeCameras;
    //DeviceList<Camera> cameras;
    //DeviceList<RangeSensor> rangeSensors;
    PointCloudIORTC* pointCloudIORTC;
    string componentName;
    vector<string> pointCloudPortNames;
    //vector<string> rangeSensorPortNames;
    //vector<string> cameraPortNames;

    OldRTMPointCloudIOItemImpl(OldRTMPointCloudIOItem* self);
    OldRTMPointCloudIOItemImpl(OldRTMPointCloudIOItem* self, const OldRTMPointCloudIOItemImpl& org);
    ~OldRTMPointCloudIOItemImpl();

    bool onComponentNamePropertyChanged(const string& name);
    bool onPointCloudPortNamePropertyChanged(int i, const string& name);

    bool recreateRTC();
    bool createRTC();
    void deleteRTC();
    void changeOwnerBodyItem(BodyItem* bodyItem);
    void doPutProperties(PutPropertyFunction& putProperty);
    void store(Archive& archive);
    void restore(const Archive& archive);
    bool start();
    void stop();
};

}


void OldRTMPointCloudIOItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<OldRTMPointCloudIOItem>(N_("OldRTMPointCloudIOItem"));
    ext->itemManager().addCreationPanel<OldRTMPointCloudIOItem>();
}


OldRTMPointCloudIOItem::OldRTMPointCloudIOItem()
{
    impl = new OldRTMPointCloudIOItemImpl(this);
    bodyItem = 0;
}


OldRTMPointCloudIOItemImpl::OldRTMPointCloudIOItemImpl(OldRTMPointCloudIOItem* self)
    : self(self),
      mv(MessageView::instance())
{
    pointCloudIORTC = 0;
    body = 0;
}


OldRTMPointCloudIOItem::OldRTMPointCloudIOItem(const OldRTMPointCloudIOItem& org)
    : ControllerItem(org)
{
    impl = new OldRTMPointCloudIOItemImpl(this, *org.impl);
    bodyItem = 0;
}


OldRTMPointCloudIOItemImpl::OldRTMPointCloudIOItemImpl(OldRTMPointCloudIOItem* self, const OldRTMPointCloudIOItemImpl& org)
    : OldRTMPointCloudIOItemImpl(self)
{
    pointCloudIORTC = 0;
    body = 0;
}


OldRTMPointCloudIOItem::~OldRTMPointCloudIOItem()
{
    delete impl;
}


OldRTMPointCloudIOItemImpl::~OldRTMPointCloudIOItemImpl()
{
    deleteRTC();
}


void OldRTMPointCloudIOItem::onPositionChanged()
{
    BodyItem* ownerBodyItem = findOwnerItem<BodyItem>();
    if(ownerBodyItem){
        if(bodyItem != ownerBodyItem){
            impl->changeOwnerBodyItem(ownerBodyItem);
            bodyItem = ownerBodyItem;
        }
    }else{
        impl->deleteRTC();
        impl->body = 0;
        bodyItem = 0;
    }
}


void OldRTMPointCloudIOItemImpl::changeOwnerBodyItem(BodyItem* bodyItem)
{
    if(componentName.empty()){
        componentName = self->name();
    }

    body = bodyItem->body();
    rangeCameras = body->devices<RangeCamera>();
    pointCloudPortNames.resize(rangeCameras.size());
    for(size_t i=0; i<rangeCameras.size(); i++){
        if(pointCloudPortNames[i].empty())
            pointCloudPortNames[i] = rangeCameras[i]->name();
    }

   //self->notifyUpdate();

    deleteRTC();
    createRTC();
}


void OldRTMPointCloudIOItem::onDisconnectedFromRoot()
{
    impl->deleteRTC();
}


bool OldRTMPointCloudIOItemImpl::createRTC()
{
    RTC::Manager* rtcManager = &RTC::Manager::instance();
    PointCloudIORTC::registerFactory(rtcManager, componentName.c_str());

    boost::format param(
        "OldPointCloudIO?"
        "instance_name=%1%&"
        "exec_cxt.periodic.type=ChoreonoidExecutionContext&"
        "exec_cxt.periodic.rate=1000000");
    
    RTC::RtcBase* rtc = createManagedRTC(str(param % componentName).c_str());
    if(!rtc){
        MessageView::instance()->putln(fmt(_("RTC \"%1%\" cannot be created.")) % componentName);
        return false;
    }
    MessageView::instance()->putln(fmt(_("RTC \"%1%\" has been created.")) % componentName);

    pointCloudIORTC = dynamic_cast<PointCloudIORTC*>(rtc);
    pointCloudIORTC->createPort(rangeCameras, pointCloudPortNames);

    execContext = OpenRTM::ExtTrigExecutionContextService::_nil();
    isChoreonoidExecutionContext = true;
    RTC::ExecutionContextList_var eclist = rtc->get_owned_contexts();
    for(CORBA::ULong i=0; i < eclist->length(); ++i){
        if(!CORBA::is_nil(eclist[i])){
            execContext = OpenRTM::ExtTrigExecutionContextService::_narrow(eclist[i]);
            break;
        }
    }

    return true;
}


void OldRTMPointCloudIOItemImpl::deleteRTC()
{
    if(pointCloudIORTC){
        pointCloudIORTC->exit();
        RTC::Manager::instance().cleanupComponents();
        pointCloudIORTC = 0;
    }
}


bool OldRTMPointCloudIOItemImpl::recreateRTC()
{
    if(body){
        deleteRTC();
        return createRTC();
    }

    return true;
}


Item* OldRTMPointCloudIOItem::doDuplicate() const
{
    return new OldRTMPointCloudIOItem(*this);
}


void OldRTMPointCloudIOItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void OldRTMPointCloudIOItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("ComponentName"), componentName,
            [this](const string& name){ return onComponentNamePropertyChanged(name); });

    for(size_t i=0; i < pointCloudPortNames.size(); i++){
        putProperty(_("PointCloudPortName")+to_string(i), pointCloudPortNames[i],
            [this, i](const string& name){ return onPointCloudPortNamePropertyChanged(i, name); });
    }
}


bool OldRTMPointCloudIOItemImpl::onComponentNamePropertyChanged(const string& name)
{
    componentName = name;

    recreateRTC();
    return true;
}


bool OldRTMPointCloudIOItemImpl::onPointCloudPortNamePropertyChanged(int i, const string& name)
{
    pointCloudPortNames[i] = name;

    recreateRTC();
    return true;

}


bool OldRTMPointCloudIOItem::store(Archive& archive)
{
    impl->store(archive);
    return true;
}


void OldRTMPointCloudIOItemImpl::store(Archive& archive)
{
    archive.write("componentName", componentName);

    Listing& list0 = *archive.createFlowStyleListing("pointCloudPortNames");
    for(size_t i=0; i<pointCloudPortNames.size(); i++){
        list0.append(pointCloudPortNames[i]);
    }
}


bool OldRTMPointCloudIOItem::restore(const Archive& archive)
{
    impl->restore(archive);
    return true;
}


void OldRTMPointCloudIOItemImpl::restore(const Archive& archive)
{
    archive.read("componentName", componentName);

    const Listing& list0 = *archive.findListing("pointCloudPortNames");
    if(list0.isValid()){
        pointCloudPortNames.clear();
        for(int i=0; i < list0.size(); i++)
            pointCloudPortNames.push_back(list0[i]);
    }
}


bool OldRTMPointCloudIOItem::start()
{
    return impl->start();
}


bool OldRTMPointCloudIOItemImpl::start()
{
    bool isReady = false;

    if(pointCloudIORTC){
        if(!CORBA::is_nil(execContext)){
            RTC::ReturnCode_t result = RTC::RTC_OK;
            RTC::LifeCycleState state = execContext->get_component_state(pointCloudIORTC->getObjRef());
            if(state == RTC::ERROR_STATE){
                result = execContext->reset_component(pointCloudIORTC->getObjRef());
                execContext->tick();
            } else if(state == RTC::ACTIVE_STATE){
                result = execContext->deactivate_component(pointCloudIORTC->getObjRef());
                execContext->tick();
            }
            if(result == RTC::RTC_OK){
                result = execContext->activate_component(pointCloudIORTC->getObjRef());
                execContext->tick();
            }
            if(result == RTC::RTC_OK){
                isReady = true;
            }
        }
    }

    return isReady;
}


double OldRTMPointCloudIOItem::timeStep() const
{
    return 0.0;
}


void OldRTMPointCloudIOItem::input()
{

}


bool OldRTMPointCloudIOItem::control()
{
    if(impl->isChoreonoidExecutionContext){
        impl->execContext->tick();
    }
    return true;
}


void OldRTMPointCloudIOItem::output()
{

}


void OldRTMPointCloudIOItem::stop()
{
    impl->stop();
}


void OldRTMPointCloudIOItemImpl::stop()
{
    RTC::LifeCycleState state = execContext->get_component_state(pointCloudIORTC->getObjRef());
    if(state == RTC::ERROR_STATE){
        execContext->reset_component(pointCloudIORTC->getObjRef());
    } else {
        execContext->deactivate_component(pointCloudIORTC->getObjRef());
    }
    execContext->tick();
}
