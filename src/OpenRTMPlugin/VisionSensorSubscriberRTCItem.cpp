/**
   \author Shizuko Hattori
   \author Shin'ichiro Nakaoka
*/

#include "VisionSensorSubscriberRTCItem.h"
#include <cnoid/BodyItem>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/Config>
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

class CameraImageInput : public Referenced
{
public:
    RTC::InPort<Img::TimedCameraImage> port;
    Img::TimedCameraImage timedCameraImage;
    CameraPtr camera;

    CameraImageInput(Camera* camera)
        : port(camera->name().c_str(), timedCameraImage),
          camera(camera) { }
};
typedef ref_ptr<CameraImageInput> CameraImageInputPtr;


class PointCloudInput : public Referenced
{
public:
    RTC::InPort<RTC::PointCloud> port;
    RTC::PointCloud pointCloud;
    RangeCameraPtr rangeCamera;

    PointCloudInput(RangeCamera* rangeCamera)
        : port((rangeCamera->name() + "-depth").c_str(), pointCloud),
          rangeCamera(rangeCamera) { }
};
typedef ref_ptr<PointCloudInput> PointCloudInputPtr;


class RangeDataInput : public Referenced
{
public:
    RTC::InPort<RTC::RangeData> port;
    RTC::RangeData timedRangeData;
    RangeSensorPtr rangeSensor;

    RangeDataInput(RangeSensor* rangeSensor)
        : port(rangeSensor->name().c_str(), timedRangeData),
          rangeSensor(rangeSensor) { }
};
typedef ref_ptr<RangeDataInput> RangeDataInputPtr;


class SubscriberRTC : public RTC::DataFlowComponentBase
{
public:
    vector<CameraImageInputPtr> cameraImageInputs;
    vector<PointCloudInputPtr> pointCloudInputs;
    vector<RangeDataInputPtr> rangeDataInputs;
    
    SubscriberRTC(RTC::Manager* manager);
    ~SubscriberRTC(){ };

    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
};

}


SubscriberRTC::SubscriberRTC(RTC::Manager* manager)
    : DataFlowComponentBase(manager)
{

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
    for(auto& input : cameraImageInputs){
        if(input->port.isNew()){
            do {
                input->port.read();
            } while(input->port.isNew());

            Img::TimedCameraImage& timedCameraImage = input->timedCameraImage;

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

            Camera* camera = input->camera;
            callLater([camera, tmpImage]() mutable {
                    camera->setImage(tmpImage);
                    camera->notifyStateChange();
                });
        }
    }

    for(auto& input : rangeDataInputs){
        if(input->port.isNew()){
            do {
                input->port.read();
            }while(input->port.isNew());

            RTC::RangeData& timedRangeData = input->timedRangeData;
            double* src = timedRangeData.ranges.get_buffer();
            int numPoints = timedRangeData.ranges.length();
            std::shared_ptr<RangeSensor::RangeData> tmpRangeData = std::make_shared<vector<double>>();
            tmpRangeData->clear();
            for(int i=0; i < numPoints; i++){
                tmpRangeData->push_back(src[i]);
            }

            RangeSensor* rangeSensor = input->rangeSensor;
            callLater([rangeSensor, tmpRangeData]() mutable {
                    rangeSensor->setRangeData(tmpRangeData);
                    rangeSensor->notifyStateChange();
                });
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
    BodyItem* bodyItem;
    SubscriberRTC* subscriberRTC;
    RTC::ExecutionContext_var execContext;
    int periodicRate;

    VisionSensorSubscriberRTCItemImpl(VisionSensorSubscriberRTCItem* self);
    VisionSensorSubscriberRTCItemImpl(VisionSensorSubscriberRTCItem* self, const VisionSensorSubscriberRTCItemImpl& org);
    ~VisionSensorSubscriberRTCItemImpl();

    void setBodyItem(BodyItem* bodyItem);
    void createRTC();
    void createInPorts();
    void deleteRTC();
    bool start();
    void stop();
};

}


void VisionSensorSubscriberRTCItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<VisionSensorSubscriberRTCItem>(N_("VisionSensorSubscriberRTCItem"));
    ext->itemManager().addCreationPanel<VisionSensorSubscriberRTCItem>();

    static const char* spec[] = {
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

    RTC::Properties profile(spec);
    RTC::Manager::instance().registerFactory(
        profile, RTC::Create<SubscriberRTC>, RTC::Delete<SubscriberRTC>);
}


VisionSensorSubscriberRTCItem::VisionSensorSubscriberRTCItem()
{
    impl = new VisionSensorSubscriberRTCItemImpl(this);
}


VisionSensorSubscriberRTCItem::VisionSensorSubscriberRTCItem(const VisionSensorSubscriberRTCItem& org)
    : Item(org)
{
    impl = new VisionSensorSubscriberRTCItemImpl(this);
}


VisionSensorSubscriberRTCItemImpl::VisionSensorSubscriberRTCItemImpl(VisionSensorSubscriberRTCItem* self)
    : self(self),
      mv(MessageView::instance())
{
    bodyItem = nullptr;
    subscriberRTC = nullptr;
    execContext = RTC::ExecutionContext::_nil();
    periodicRate = 30;

    self->sigNameChanged().connect([&](const string&){ createRTC(); });
}


VisionSensorSubscriberRTCItemImpl::VisionSensorSubscriberRTCItemImpl
(VisionSensorSubscriberRTCItem* self, const VisionSensorSubscriberRTCItemImpl& org)
    : VisionSensorSubscriberRTCItemImpl(self)
{
    periodicRate = org.periodicRate;
}


VisionSensorSubscriberRTCItem::~VisionSensorSubscriberRTCItem()
{
    delete impl;
}


VisionSensorSubscriberRTCItemImpl::~VisionSensorSubscriberRTCItemImpl()
{
    deleteRTC();
}


void VisionSensorSubscriberRTCItem::onDisconnectedFromRoot()
{
    impl->deleteRTC();
}


void VisionSensorSubscriberRTCItem::onPositionChanged()
{
    impl->setBodyItem(findOwnerItem<BodyItem>());
}


void VisionSensorSubscriberRTCItemImpl::setBodyItem(BodyItem* newBodyItem)
{
    if(newBodyItem){
        if(newBodyItem != bodyItem){
            bodyItem = newBodyItem;
            deleteRTC();
            createRTC();
        }
    } else {
        deleteRTC();
        bodyItem = nullptr;
    }
}


void VisionSensorSubscriberRTCItem::setPeriodicRate(int rate)
{
    if(rate != impl->periodicRate){
        impl->periodicRate = rate;
        impl->createRTC();
    }
}

void VisionSensorSubscriberRTCItemImpl::createRTC()
{
    deleteRTC();

    if(!bodyItem){
        return;
    }
    
    boost::format param(
        "VisionSensorSubscriber?"
        "instance_name=%1%&"
        "exec_cxt.periodic.type=PeriodicExecutionContext&"
        "exec_cxt.periodic.rate=%2%");
    
    RTC::RtcBase* rtc = createManagedRTC(str(param % self->name() % periodicRate).c_str());
    if(!rtc){
        mv->putln(MessageView::ERROR, boost::format(_("RTC for \"%1%\" cannot be created.")) % self->name());
        return;
    }

    subscriberRTC = dynamic_cast<SubscriberRTC*>(rtc);
    createInPorts();

    execContext = RTC::ExecutionContext::_nil();
    RTC::ExecutionContextList_var eclist = rtc->get_owned_contexts();
    for(CORBA::ULong i=0; i < eclist->length(); ++i){
        execContext = eclist[i];
        if(!CORBA::is_nil(execContext)){
            execContext->activate_component(subscriberRTC->getObjRef());
            break;
        }
    }
}


void VisionSensorSubscriberRTCItemImpl::createInPorts()
{
    DeviceList<> devices = bodyItem->body()->devices();
    
    DeviceList<Camera> cameras(devices);

    for(size_t i=0; i < cameras.size(); ++i){
        auto camera = cameras[i];
        if(camera->imageType() != Camera::NO_IMAGE){
            CameraImageInputPtr input = new CameraImageInput(camera);
            if(subscriberRTC->addInPort(input->port.name(), input->port)){
                subscriberRTC->cameraImageInputs.push_back(input);
            }
        }
        auto rangeCamera = dynamic_pointer_cast<RangeCamera>(camera);
        if(rangeCamera){
            PointCloudInputPtr input = new PointCloudInput(rangeCamera);
            if(subscriberRTC->addInPort(input->port.name(), input->port)){
                subscriberRTC->pointCloudInputs.push_back(input);
            }
        }
    }

    DeviceList<RangeSensor> rangeSensors(devices);
    for(size_t i=0; i < rangeSensors.size(); ++i){
        auto sensor = rangeSensors[i];
        RangeDataInputPtr input = new RangeDataInput(sensor);
        if(subscriberRTC->addInPort(input->port.name(), input->port)){
            subscriberRTC->rangeDataInputs.push_back(input);
        }
    }
}


void VisionSensorSubscriberRTCItemImpl::deleteRTC()
{
    if(subscriberRTC){
        subscriberRTC->exit();
        RTC::Manager::instance().cleanupComponents();
        subscriberRTC = nullptr;
    }
}


Item* VisionSensorSubscriberRTCItem::doDuplicate() const
{
    return new VisionSensorSubscriberRTCItem(*this);
}


void VisionSensorSubscriberRTCItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty.decimals(3)(_("Periodic rate"), impl->periodicRate,
                            [&](int rate){ setPeriodicRate(rate); return true; });
}


bool VisionSensorSubscriberRTCItem::store(Archive& archive)
{
    archive.write("periodicRate", impl->periodicRate);
    return true;
}


bool VisionSensorSubscriberRTCItem::restore(const Archive& archive)
{
    archive.read("periodicRate", impl->periodicRate);
    return true;
}
