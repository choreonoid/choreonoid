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
#include <cnoid/corba/PointCloud.hh>

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

class SubscriberRTC;

class InputBase : public Referenced
{
public:
    virtual void read() = 0;
};

typedef ref_ptr<InputBase> InputBasePtr;
    
class CameraImageInput : public InputBase
{
public:
    CameraPtr camera;
    RTC::InPort<Img::TimedCameraImage> port;
    Img::TimedCameraImage timedCameraImage;

    CameraImageInput(SubscriberRTC* rtc, Camera* camera);
    void read();
    shared_ptr<Image> createImageFromRawData(Img::ImageData& srcImage, int numComponents);
    shared_ptr<Image> createImageFromImageFormat(Img::ImageData& srcImage);
};

typedef ref_ptr<CameraImageInput> CameraImageInputPtr;


class PointCloudInput : public InputBase
{
public:
    RangeCameraPtr rangeCamera;
    int pointCloudPortType;
    RTC::InPort<PointCloudTypes::PointCloud> port1;
    PointCloudTypes::PointCloud pointCloud1;
    RTC::InPort<RTC::PointCloud> port2;
    RTC::PointCloud pointCloud2;

    struct PointCloudField {
        unsigned long offset;
        PointCloudTypes::DataType type;
        unsigned long count;
    };
    std::map<string, PointCloudField> pointCloudField;

    PointCloudInput(SubscriberRTC* rtc, RangeCamera* rangeCamera, int pointCloudPortType);
    void read();
    void read1();
    float readFloatPointCloudData(unsigned char* src, PointCloudField& fe, bool is_bigendian);
    void read2();
};

typedef ref_ptr<PointCloudInput> PointCloudInputPtr;


class RangeDataInput : public InputBase
{
public:
    RangeSensorPtr rangeSensor;
    RTC::InPort<RTC::RangeData> port;
    RTC::RangeData timedRangeData;

    RangeDataInput(SubscriberRTC* rtc, RangeSensor* rangeSensor);
    void read();
};

typedef ref_ptr<RangeDataInput> RangeDataInputPtr;

class SubscriberRTC : public RTC::DataFlowComponentBase
{
public:
    vector<InputBasePtr> inputs;
    
    SubscriberRTC(RTC::Manager* manager);
    void createInPorts(Body* body, int pointCloudPortType);
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


void SubscriberRTC::createInPorts(Body* body, int pointCloudPortType)
{
    DeviceList<> devices = body->devices();
    
    DeviceList<Camera> cameras(devices);

    for(size_t i=0; i < cameras.size(); ++i){
        auto camera = cameras[i];
        if(camera->imageType() != Camera::NO_IMAGE){
            inputs.push_back(new CameraImageInput(this, camera));
        }
        auto rangeCamera = dynamic_pointer_cast<RangeCamera>(camera);
        if(rangeCamera){
            inputs.push_back(new PointCloudInput(this, rangeCamera, pointCloudPortType));
        }
    }

    DeviceList<RangeSensor> rangeSensors(devices);
    for(size_t i=0; i < rangeSensors.size(); ++i){
        inputs.push_back(new RangeDataInput(this, rangeSensors[i]));
    }
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
    for(auto& input : inputs){
        input->read();
    }
    return RTC::RTC_OK;
}


CameraImageInput::CameraImageInput(SubscriberRTC* rtc, Camera* camera)
    : camera(camera),
      port(camera->name().c_str(), timedCameraImage)
{
    rtc->addInPort(port.name(), port);
}


void CameraImageInput::read()
{
    if(port.isNew()){
        do {
            port.read();
        } while(port.isNew());
        
        std::shared_ptr<Image> image;
        auto srcImage = timedCameraImage.data.image;
        switch(srcImage.format){
        case Img::CF_RGB:
            image = createImageFromRawData(srcImage, 3);
            break;
        case Img::CF_GRAY:
            image = createImageFromRawData(srcImage, 1);
            break;

#ifndef USE_BUILTIN_CAMERA_IMAGE_IDL
        case Img::CF_JPEG:
        case Img::CF_PNG:
#else
        case Img::CF_GRAY_JPEG:
        case Img::CF_RGB_JPEG:
#endif
            image = createImageFromImageFormat(srcImage);
            break;
        default:
            break;
        }

        if(image){
            CameraPtr tmpCamera = camera;
            callLater([tmpCamera, image]() mutable {
                tmpCamera->setImage(image);
                tmpCamera->notifyStateChange();
                });
        }
    }
}


shared_ptr<Image> CameraImageInput::createImageFromRawData(Img::ImageData& srcImage, int numComponents)
{
    auto image = std::make_shared<Image>();
    image->setSize(srcImage.width, srcImage.height, numComponents);
    memcpy(image->pixels(), srcImage.raw_data.get_buffer(), srcImage.raw_data.length());
    return image;
}


shared_ptr<Image> CameraImageInput::createImageFromImageFormat(Img::ImageData& srcImage)
{
    auto image = std::make_shared<Image>();
    auto& data = srcImage.raw_data;
    QImage qImage = QImage::fromData(data.get_buffer(), data.length());
    const int w = qImage.width();
    const int h = qImage.height();
    const bool hasAlpha = qImage.hasAlphaChannel();
    if(qImage.isGrayscale()){
        image->setSize(w, h, 1);
        unsigned char* pixels = image->pixels();
        for(int y=0; y < h; ++y){
            for(int x=0; x < w; ++x){
                *pixels++ = qGray(qImage.pixel(x, y));
            }
        }
    } else {
        image->setSize(w, h, hasAlpha ? 4 : 3);
        unsigned char* pixels = image->pixels();
        for(int y=0; y < h; ++y){
            for(int x=0; x < w; ++x){
                QRgb rgb = qImage.pixel(x, y);
                *pixels++ = qRed(rgb);
                *pixels++ = qGreen(rgb);
                *pixels++ = qBlue(rgb);
                if(hasAlpha){
                    *pixels++ = qAlpha(rgb);
                }
            }
        }
    }
    return image;
}


PointCloudInput::PointCloudInput(SubscriberRTC* rtc, RangeCamera* rangeCamera, int pointCloudPortType)
    : rangeCamera(rangeCamera),
      pointCloudPortType(pointCloudPortType),
      port1((rangeCamera->name() + "-depth").c_str(), pointCloud1),
      port2((rangeCamera->name() + "-depth").c_str(), pointCloud2)
{
    if(pointCloudPortType == VisionSensorSubscriberRTCItem::POINT_CLOUD_TYPES_POINT_CLOUD_TYPE){

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
        
        rtc->addInPort(port1.name(), port1);
        
    } else if(pointCloudPortType == VisionSensorSubscriberRTCItem::RTC_POINT_CLOUD_TYPE){
        rtc->addInPort(port2.name(), port2);
    }
}


void PointCloudInput::read()
{
    if(pointCloudPortType == VisionSensorSubscriberRTCItem::POINT_CLOUD_TYPES_POINT_CLOUD_TYPE){
        read1();
    } else if(pointCloudPortType == VisionSensorSubscriberRTCItem::RTC_POINT_CLOUD_TYPE){
        read2();
    }
}


void PointCloudInput::read1()
{
    if(port1.isNew()){
        do {
            port1.read();
        } while(port1.isNew());
    
        bool rgb = false;
        for(size_t i=0; i < pointCloud1.fields.length(); ++i){
            string name = string(pointCloud1.fields[i].name);
            pointCloudField[name].offset = pointCloud1.fields[i].offset;
            pointCloudField[name].type = pointCloud1.fields[i].data_type;
            pointCloudField[name].count = pointCloud1.fields[i].count;
            if(i==5){
                rgb = true;
            }
        }

        int n = pointCloud1.height * pointCloud1.width;
        int m = pointCloud1.data.length() / pointCloud1.point_step;
        int numPoints = std::min(n, m);
        unsigned char* src = (unsigned char*)pointCloud1.data.get_buffer();
        auto tmpPoints = std::make_shared<vector<Vector3f>>();
        std::shared_ptr<Image> tmpImage;
        unsigned char* pixels = nullptr;
        if(rgb){
            tmpImage = std::make_shared<Image>();
            tmpImage->setSize(pointCloud1.width, pointCloud1.height, 3);
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
        
        for(int i=0; i < numPoints; ++i, src+=pointCloud1.point_step){
            Vector3f point;
            point.x() = readFloatPointCloudData(&src[fex.offset], fex, pointCloud1.is_bigendian );
            point.y() = readFloatPointCloudData(&src[fey.offset], fey, pointCloud1.is_bigendian );
            point.z() = readFloatPointCloudData(&src[fez.offset], fez, pointCloud1.is_bigendian );
            tmpPoints->push_back(point);
            
            if(rgb && pixels){
                pixels[0] = src[fer.offset];
                pixels[1] = src[feg.offset];
                pixels[2] = src[feb.offset];
                pixels += 3;
            }
        }
        
        if(!rgb || !pixels){
            tmpImage.reset();
        }

        RangeCameraPtr tmpRangeCamera = rangeCamera;
        callLater([tmpRangeCamera, tmpPoints, tmpImage]() mutable {
                tmpRangeCamera->setPoints(tmpPoints);
                if(tmpImage){
                    tmpRangeCamera->setImage(tmpImage);
                }
                tmpRangeCamera->notifyStateChange();
            });
    }
}


float PointCloudInput::readFloatPointCloudData(unsigned char* src, PointCloudField& fe, bool is_bigendian)
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


void PointCloudInput::read2()
{
    if(port2.isNew()){
        do {
            port2.read();
        } while(port2.isNew());

        const RTC::PointCloudPointList& pcPoints = pointCloud2.points;
        const int numPoints = pcPoints.length();
        auto tmpPoints = std::make_shared<RangeCamera::PointData>(numPoints);
        
        for(int i=0; i < numPoints; ++i){
            Vector3f& p = (*tmpPoints)[i];
            const RTC::Point3D& point = pcPoints[i].point;
            p.x() = point.x;
            p.y() = point.y;
            p.z() = point.z;
        }

        RangeCameraPtr tmpRangeCamera = rangeCamera;
        callLater([tmpRangeCamera, tmpPoints]() mutable {
                tmpRangeCamera->setPoints(tmpPoints);
                tmpRangeCamera->notifyStateChange();
            });
    }
}


RangeDataInput::RangeDataInput(SubscriberRTC* rtc, RangeSensor* rangeSensor)
    : rangeSensor(rangeSensor),
      port(rangeSensor->name().c_str(), timedRangeData)
{
    rtc->addInPort(port.name(), port);
}


void RangeDataInput::read()
{
    if(port.isNew()){
        do {
            port.read();
        } while(port.isNew());
            
        double* src = timedRangeData.ranges.get_buffer();
        int numPoints = timedRangeData.ranges.length();
        std::shared_ptr<RangeSensor::RangeData> tmpRangeData = std::make_shared<vector<double>>();
        tmpRangeData->clear();
        for(int i=0; i < numPoints; i++){
            tmpRangeData->push_back(src[i]);
        }
            
        RangeSensorPtr tmpRangeSensor = rangeSensor;
        callLater([tmpRangeSensor, tmpRangeData]() mutable {
                tmpRangeSensor->setRangeData(tmpRangeData);
                tmpRangeSensor->notifyStateChange();
            });
    }
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
    Selection pointCloudPortType;

    VisionSensorSubscriberRTCItemImpl(VisionSensorSubscriberRTCItem* self);
    VisionSensorSubscriberRTCItemImpl(VisionSensorSubscriberRTCItem* self, const VisionSensorSubscriberRTCItemImpl& org);
    ~VisionSensorSubscriberRTCItemImpl();

    void setBodyItem(BodyItem* bodyItem);
    void createRTC();
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
      mv(MessageView::instance()),
      pointCloudPortType(VisionSensorSubscriberRTCItem::N_POINT_CLOUD_PORT_TYPES, CNOID_GETTEXT_DOMAIN_NAME)
{
    bodyItem = nullptr;
    subscriberRTC = nullptr;
    execContext = RTC::ExecutionContext::_nil();
    periodicRate = 30;

    pointCloudPortType.setSymbol(
        VisionSensorSubscriberRTCItem::POINT_CLOUD_TYPES_POINT_CLOUD_TYPE,
        "PointCloudTypes::PointCloud");
    pointCloudPortType.setSymbol(
        VisionSensorSubscriberRTCItem::RTC_POINT_CLOUD_TYPE,
        "RTC::PointCloud");
    pointCloudPortType.select(VisionSensorSubscriberRTCItem::POINT_CLOUD_TYPES_POINT_CLOUD_TYPE);
    
    self->sigNameChanged().connect([&](const string&){ createRTC(); });
}


VisionSensorSubscriberRTCItemImpl::VisionSensorSubscriberRTCItemImpl
(VisionSensorSubscriberRTCItem* self, const VisionSensorSubscriberRTCItemImpl& org)
    : VisionSensorSubscriberRTCItemImpl(self)
{
    periodicRate = org.periodicRate;
    pointCloudPortType = org.pointCloudPortType;
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


void VisionSensorSubscriberRTCItem::setPointCloudPortType(int type)
{
    if(type != impl->pointCloudPortType.which()){
        impl->pointCloudPortType.select(type);
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
    subscriberRTC->createInPorts(bodyItem->body(), pointCloudPortType.which());

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

    putProperty(_("Point cloud port type"), impl->pointCloudPortType,
                [&](int which){ setPointCloudPortType(which); return true; });
}


bool VisionSensorSubscriberRTCItem::store(Archive& archive)
{
    archive.write("periodicRate", impl->periodicRate);
    archive.write("pointCloudPortType", impl->pointCloudPortType.selectedSymbol(), DOUBLE_QUOTED);
    return true;
}


bool VisionSensorSubscriberRTCItem::restore(const Archive& archive)
{
    archive.read("periodicRate", impl->periodicRate);

    string type;
    if(archive.read("pointCloudPortType", type)){
        impl->pointCloudPortType.select(type);
    }
    
    return true;
}
