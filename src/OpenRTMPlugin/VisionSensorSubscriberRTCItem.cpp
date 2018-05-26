/**
   \author Shizuko Hattori
   \author Shin'ichiro Nakaoka
*/

#include "VisionSensorSubscriberRTCItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/Config>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <cnoid/MessageView>
#include <cnoid/OpenRTMUtil>
#include <cnoid/LazyCaller>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>
#include <rtm/idl/InterfaceDataTypes.hh>

#ifdef USE_BUILTIN_CAMERA_IMAGE_IDL
# include "deprecated/corba/CameraImage.hh"
#else
# ifdef WIN32
#  include <rtm/idl/CameraCommonInterface.hh>
# else
#  include <rtm/ext/CameraCommonInterface.hh>
# endif
#endif

#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class CameraInPort
{
public:
    RTC::InPort<Img::TimedCameraImage> inPort;
    Img::TimedCameraImage timedCameraImage;
    string portName;
    CameraPtr camera;

    CameraInPort(Camera* camera, const string& name)
        : inPort(name.c_str(), timedCameraImage),
          portName(name),
          camera(camera) { }
};
typedef std::shared_ptr<CameraInPort> CameraInPortPtr;

class RangeDataInPort
{
public:
    RTC::InPort<RTC::RangeData> inPort;
    RTC::RangeData timedRangeData;
    string portName;
    RangeSensorPtr rangeSensor;

    RangeDataInPort(RangeSensor* rangeSensor, const string& name)
        : inPort(name.c_str(), timedRangeData),
          portName(name),
          rangeSensor(rangeSensor) { }
};
typedef std::shared_ptr<RangeDataInPort> RangeDataInPortPtr;


class SubscriberRTC : public RTC::DataFlowComponentBase
{
public:
    typedef map<string, RangeDataInPortPtr> RangeDataInPortMap;
    RangeDataInPortMap rangeDataInPorts;

    typedef map<string, CameraInPortPtr> CameraInPortMap;
    CameraInPortMap cameraInPorts;

    static void registerFactory(RTC::Manager* manager, const char* componentTypeName);

    SubscriberRTC(RTC::Manager* manager);
    ~SubscriberRTC(){ };

    bool createPort(
        DeviceList<RangeSensor> rangeSensors, vector<string> rangeSensorPortNames,
        DeviceList<Camera> cameras, vector<string> cameraPortNames );

    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
};

}


void SubscriberRTC::registerFactory(RTC::Manager* manager, const char* componentTypeName)
{
  static const char* pointCloudIORTC_spec[] = {
    "implementation_id", "VisionSensorSubscriber",
    "type_name",         "VisionSensorSubscriber",
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
                           RTC::Create<SubscriberRTC>,
                           RTC::Delete<SubscriberRTC>);
}


SubscriberRTC::SubscriberRTC(RTC::Manager* manager)
    : DataFlowComponentBase(manager)
{

}


bool SubscriberRTC::createPort(
    DeviceList<RangeSensor> rangeSensors, vector<string> rangeSensorPortNames,
    DeviceList<Camera> cameras, vector<string> cameraPortNames)
{
    bool ret = true;

    for(size_t i=0; i<rangeSensors.size(); i++){
        RangeDataInPort* rangeDataInPort = new RangeDataInPort(rangeSensors[i], rangeSensorPortNames[i]);
        if(!addInPort(rangeDataInPort->portName.c_str(), rangeDataInPort->inPort))
            ret = false;
        else
            rangeDataInPorts.insert(make_pair(rangeDataInPort->portName, RangeDataInPortPtr(rangeDataInPort)));
    }

    for(size_t i=0; i < cameras.size(); i++){
        CameraInPort* cameraInPort = new CameraInPort(cameras[i], cameraPortNames[i]);
        if(!addInPort(cameraInPort->portName.c_str(), cameraInPort->inPort))
            ret = false;
        else
            cameraInPorts.insert(make_pair(cameraInPort->portName, CameraInPortPtr(cameraInPort)));
    }

    return ret;
}


RTC::ReturnCode_t SubscriberRTC::onInitialize()
{
    return RTC::RTC_OK;
}


RTC::ReturnCode_t SubscriberRTC::onActivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}

RTC::ReturnCode_t SubscriberRTC::onDeactivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}


RTC::ReturnCode_t SubscriberRTC::onExecute(RTC::UniqueId ec_id)
{
    for(RangeDataInPortMap::iterator it = rangeDataInPorts.begin();
            it != rangeDataInPorts.end(); it++){
        RTC::InPort<RTC::RangeData>& inport_ = it->second->inPort;
        if(inport_.isNew()){
            do {
                inport_.read();
            }while(inport_.isNew());

            RTC::RangeData& timedRangeData = it->second->timedRangeData;
            RangeSensor* rangeSensor = it->second->rangeSensor;

            double* src = timedRangeData.ranges.get_buffer();
            int numPoints = timedRangeData.ranges.length();
            std::shared_ptr<RangeSensor::RangeData> tmpRangeData = std::make_shared< vector<double> >();
            tmpRangeData->clear();
            for(int i=0; i < numPoints; i++)
                tmpRangeData->push_back(src[i]);

            rangeSensor->setRangeData(tmpRangeData);

            callLater( [rangeSensor](){rangeSensor->notifyStateChange();} );
        }
    }

    for(CameraInPortMap::iterator it = cameraInPorts.begin();
            it != cameraInPorts.end(); it++){
        RTC::InPort<Img::TimedCameraImage>& inport_ = it->second->inPort;
        if(inport_.isNew()){
            do {
                inport_.read();
            }while(inport_.isNew());

            Img::TimedCameraImage& timedCameraImage = it->second->timedCameraImage;
            Camera* camera = it->second->camera;

            int numComponents;
            switch(timedCameraImage.data.image.format){
            case Img::CF_GRAY :
                numComponents = 1;
                break;
            case Img::CF_RGB :
                numComponents = 3;
                break;
            case Img::CF_UNKNOWN :
            default :
                numComponents = 0;
                break;
            }

            std::shared_ptr<Image> tmpImage = std::make_shared<Image>();
            tmpImage->setSize(timedCameraImage.data.image.width, timedCameraImage.data.image.height, numComponents);
            size_t length = timedCameraImage.data.image.raw_data.length();
            memcpy(tmpImage->pixels(), timedCameraImage.data.image.raw_data.get_buffer(), length);
            camera->setImage(tmpImage);

            callLater( [camera](){camera->notifyStateChange();} );
        }
    }

    return RTC::RTC_OK;
}

namespace cnoid {

class VisionSensorSubscriberRTCItemImpl
{
public:
    VisionSensorSubscriberRTCItem* self;
    MessageView* mv;
    OpenRTM::ExtTrigExecutionContextService_var execContext;
    bool isSimulationExecutionContext;

    Body* body;
    DeviceList<Camera> cameras;
    DeviceList<RangeSensor> rangeSensors;
    SubscriberRTC* subscriberRTC;
    string componentName;
    vector<string> rangeSensorPortNames;
    vector<string> cameraPortNames;

    VisionSensorSubscriberRTCItemImpl(VisionSensorSubscriberRTCItem* self);
    VisionSensorSubscriberRTCItemImpl(VisionSensorSubscriberRTCItem* self, const VisionSensorSubscriberRTCItemImpl& org);
    ~VisionSensorSubscriberRTCItemImpl();

    bool onComponentNamePropertyChanged(const string& name);
    bool onCameraPortNamePropertyChanged(int i, const string& name);
    bool onRangeSensorPortNamePropertyChanged(int i, const string& name);

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


void VisionSensorSubscriberRTCItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<VisionSensorSubscriberRTCItem>(N_("VisionSensorSubscriberRTCItem"));
    ext->itemManager().addCreationPanel<VisionSensorSubscriberRTCItem>();
}


VisionSensorSubscriberRTCItem::VisionSensorSubscriberRTCItem()
{
    impl = new VisionSensorSubscriberRTCItemImpl(this);
    bodyItem = 0;
}


VisionSensorSubscriberRTCItemImpl::VisionSensorSubscriberRTCItemImpl(VisionSensorSubscriberRTCItem* self)
    : self(self),
      mv(MessageView::instance())
{
    subscriberRTC = 0;
    body = 0;
}


VisionSensorSubscriberRTCItem::VisionSensorSubscriberRTCItem(const VisionSensorSubscriberRTCItem& org)
    : ControllerItem(org)
{
    impl = new VisionSensorSubscriberRTCItemImpl(this, *org.impl);
    bodyItem = 0;
}


VisionSensorSubscriberRTCItemImpl::VisionSensorSubscriberRTCItemImpl(VisionSensorSubscriberRTCItem* self, const VisionSensorSubscriberRTCItemImpl& org)
    : VisionSensorSubscriberRTCItemImpl(self)
{
    subscriberRTC = 0;
    body = 0;
}


VisionSensorSubscriberRTCItem::~VisionSensorSubscriberRTCItem()
{
    delete impl;
}


VisionSensorSubscriberRTCItemImpl::~VisionSensorSubscriberRTCItemImpl()
{
    deleteRTC();
}


void VisionSensorSubscriberRTCItem::onPositionChanged()
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


void VisionSensorSubscriberRTCItemImpl::changeOwnerBodyItem(BodyItem* bodyItem)
{
    body = bodyItem->body();
    cameras.clear();
    DeviceList<Camera> cameras0 = body->devices<Camera>();
    for(size_t i=0; i<cameras0.size(); i++){
        if(cameras0[i]->imageType() != Camera::NO_IMAGE)
            cameras.push_back(cameras0[i]);
    }
    rangeSensors = body->devices<RangeSensor>();

    if(componentName.empty())
        componentName = self->name();

    rangeSensorPortNames.resize(rangeSensors.size());
    for(size_t i=0; i<rangeSensors.size(); i++){
        if(rangeSensorPortNames[i].empty())
            rangeSensorPortNames[i] = rangeSensors[i]->name();
    }

    cameraPortNames.resize(cameras.size());
    for(size_t i=0; i<cameras.size(); i++){
        if(cameraPortNames[i].empty())
            cameraPortNames[i] = cameras[i]->name() + "_image";
    }

   //self->notifyUpdate();

    deleteRTC();
    createRTC();
}


void VisionSensorSubscriberRTCItem::onDisconnectedFromRoot()
{
    impl->deleteRTC();
}


bool VisionSensorSubscriberRTCItemImpl::createRTC()
{
    RTC::Manager* rtcManager = &RTC::Manager::instance();
    SubscriberRTC::registerFactory(rtcManager, componentName.c_str());

    /// --- create RTC ---
    boost::format param(
        "VisionSensorSubscriber?"
        "instance_name=%1%&"
        "exec_cxt.periodic.type=SimulationExecutionContext&"
        "exec_cxt.periodic.rate=1000000");
    
    RTC::RtcBase* rtc = createManagedRTC(str(param % componentName).c_str());
    if(!rtc){
        MessageView::instance()->putln(fmt(_("RTC \"%1%\" cannot be created.")) % componentName);
        return false;
    }
    MessageView::instance()->putln(fmt(_("RTC \"%1%\" has been created.")) % componentName);

    subscriberRTC = dynamic_cast<SubscriberRTC*>(rtc);
    subscriberRTC->createPort(rangeSensors, rangeSensorPortNames, cameras, cameraPortNames );

    execContext = OpenRTM::ExtTrigExecutionContextService::_nil();
    isSimulationExecutionContext = true;
    RTC::ExecutionContextList_var eclist = rtc->get_owned_contexts();
    for(CORBA::ULong i=0; i < eclist->length(); ++i){
        if(!CORBA::is_nil(eclist[i])){
            execContext = OpenRTM::ExtTrigExecutionContextService::_narrow(eclist[i]);
            break;
        }
    }

    return true;
}


void VisionSensorSubscriberRTCItemImpl::deleteRTC()
{
    if(subscriberRTC){
        subscriberRTC->exit();
        RTC::Manager::instance().cleanupComponents();
        subscriberRTC = 0;
    }
}


bool VisionSensorSubscriberRTCItemImpl::recreateRTC()
{
    if(body){
        deleteRTC();
        return createRTC();
    }

    return true;
}


Item* VisionSensorSubscriberRTCItem::doDuplicate() const
{
    return new VisionSensorSubscriberRTCItem(*this);
}


void VisionSensorSubscriberRTCItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void VisionSensorSubscriberRTCItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("ComponentName"), componentName,
            [this](const string& name){ return onComponentNamePropertyChanged(name); });

    for(size_t i=0; i < cameraPortNames.size(); i++){
        putProperty(_("CameraPortName")+to_string(i), cameraPortNames[i],
            [this, i](const string& name){ return onCameraPortNamePropertyChanged(i, name); });
        }
    for(size_t i=0; i < rangeSensorPortNames.size(); i++){
        putProperty(_("rangeSensorPortName")+to_string(i), rangeSensorPortNames[i],
            [this, i](const string& name){ return onRangeSensorPortNamePropertyChanged(i, name); });
    }
}


bool VisionSensorSubscriberRTCItemImpl::onComponentNamePropertyChanged(const string& name)
{
    componentName = name;

    recreateRTC();
    return true;
}


bool VisionSensorSubscriberRTCItemImpl::onCameraPortNamePropertyChanged(int i, const string& name)
{
    cameraPortNames[i] = name;

    recreateRTC();
    return true;
}


bool VisionSensorSubscriberRTCItemImpl::onRangeSensorPortNamePropertyChanged(int i, const string& name)
{
    rangeSensorPortNames[i] = name;

    recreateRTC();
    return true;

}


bool VisionSensorSubscriberRTCItem::store(Archive& archive)
{
    impl->store(archive);
    return true;
}


void VisionSensorSubscriberRTCItemImpl::store(Archive& archive)
{
    archive.write("componentName", componentName);

    Listing& list1 = *archive.createFlowStyleListing("rangeSensorPortNames");
    for(size_t i=0; i<rangeSensorPortNames.size(); i++){
        list1.append(rangeSensorPortNames[i]);
    }

    Listing& list2 = *archive.createFlowStyleListing("cameraPortNames");
    for(size_t i=0; i<cameraPortNames.size(); i++){
        list2.append(cameraPortNames[i]);
    }
}

bool VisionSensorSubscriberRTCItem::restore(const Archive& archive)
{
    impl->restore(archive);
    return true;
}


void VisionSensorSubscriberRTCItemImpl::restore(const Archive& archive)
{
    archive.read("componentName", componentName);

    const Listing& list1 = *archive.findListing("rangeSensorPortNames");
    if(list1.isValid()){
        rangeSensorPortNames.clear();
        for(int i=0; i < list1.size(); i++)
            rangeSensorPortNames.push_back(list1[i]);
    }

    const Listing& list2 = *archive.findListing("cameraPortNames");
    if(list2.isValid()){
        cameraPortNames.clear();
        for(int i=0; i < list2.size(); i++)
            cameraPortNames.push_back(list2[i]);
    }

}


bool VisionSensorSubscriberRTCItem::start()
{
    return impl->start();
}


bool VisionSensorSubscriberRTCItemImpl::start()
{
    bool isReady = false;

    if(subscriberRTC){
        if(!CORBA::is_nil(execContext)){
            RTC::ReturnCode_t result = RTC::RTC_OK;
            RTC::LifeCycleState state = execContext->get_component_state(subscriberRTC->getObjRef());
            if(state == RTC::ERROR_STATE){
                result = execContext->reset_component(subscriberRTC->getObjRef());
                execContext->tick();
            } else if(state == RTC::ACTIVE_STATE){
                result = execContext->deactivate_component(subscriberRTC->getObjRef());
                execContext->tick();
            }
            if(result == RTC::RTC_OK){
                result = execContext->activate_component(subscriberRTC->getObjRef());
                execContext->tick();
            }
            if(result == RTC::RTC_OK){
                isReady = true;
            }
        }
    }

    return isReady;
}


double VisionSensorSubscriberRTCItem::timeStep() const
{
    return 0.0;
}


void VisionSensorSubscriberRTCItem::input()
{

}


bool VisionSensorSubscriberRTCItem::control()
{
    if(impl->isSimulationExecutionContext){
        impl->execContext->tick();
    }
    return true;
}


void VisionSensorSubscriberRTCItem::output()
{

}


void VisionSensorSubscriberRTCItem::stop()
{
    impl->stop();
}


void VisionSensorSubscriberRTCItemImpl::stop()
{
    RTC::LifeCycleState state = execContext->get_component_state(subscriberRTC->getObjRef());
    if(state == RTC::ERROR_STATE){
        execContext->reset_component(subscriberRTC->getObjRef());
    } else {
        execContext->deactivate_component(subscriberRTC->getObjRef());
    }
    execContext->tick();
}
