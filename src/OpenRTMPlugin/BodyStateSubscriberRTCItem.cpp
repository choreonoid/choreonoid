/**
   \author Shizuko Hattori
   \author Shin'ichiro Nakaoka
*/

#include "BodyStateSubscriberRTCItem.h"
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
#include <fmt/format.h>
#include <mutex>

#ifdef USE_BUILTIN_CAMERA_IMAGE_IDL
# include "deprecated/corba/CameraImage.hh"
#else
# ifdef _WIN32
#  include <rtm/idl/CameraCommonInterface.hh>
# else
#  include <rtm/ext/CameraCommonInterface.hh>
# endif
#endif

#if defined(_WIN32) && defined(ERROR)
#undef ERROR
#endif

#include "gettext.h"

#if defined(ERROR)
#undef ERROR
#endif

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

// For the backward compatibility
class VisionSensorSubscriberRTCItem : public BodyStateSubscriberRTCItem
{
public:
    VisionSensorSubscriberRTCItem() { }
    VisionSensorSubscriberRTCItem(const VisionSensorSubscriberRTCItem& org)
        : BodyStateSubscriberRTCItem(org) { }
    virtual Item* doDuplicate() const { return new VisionSensorSubscriberRTCItem(*this); };
};

class SubscriberRTC;

class InputBase : public Referenced
{
public:
    virtual void read() = 0;
};

typedef ref_ptr<InputBase> InputBasePtr;


class KinematicStateInput : public InputBase
{
public:
    
    BodyItem* bodyItem;
    RTC::InPort<RTC::TimedDoubleSeq> qin;
    RTC::TimedDoubleSeq q;
    vector<double> qtmp;
    LazyCaller updateKinematicStateCaller;
    std::mutex kinematicStateMutex;

    KinematicStateInput(SubscriberRTC* rtc, BodyItem* bodyItem);
    virtual void read() override;
    void updateKinematicState();
};

    
class CameraImageInput : public InputBase
{
public:
    CameraPtr camera;
    RTC::InPort<Img::TimedCameraImage> port;
    Img::TimedCameraImage timedCameraImage;

    CameraImageInput(SubscriberRTC* rtc, Camera* camera);
    virtual void read() override;
    shared_ptr<Image> createImageFromRawData(Img::ImageData& srcImage, int numComponents);
    shared_ptr<Image> createImageFromImageFormat(Img::ImageData& srcImage);
};


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
    virtual void read() override;
    void read1();
    float readFloatPointCloudData(unsigned char* src, PointCloudField& fe, bool is_bigendian);
    void read2();
};


class RangeDataInput : public InputBase
{
public:
    RangeSensorPtr rangeSensor;
    RTC::InPort<RTC::RangeData> port;
    RTC::RangeData timedRangeData;

    RangeDataInput(SubscriberRTC* rtc, RangeSensor* rangeSensor);
    virtual void read() override;
};


class SubscriberRTC : public RTC::DataFlowComponentBase
{
public:
    vector<InputBasePtr> inputs;
    
    SubscriberRTC(RTC::Manager* manager);
    void createInPorts(BodyItem* bodyItem, int pointCloudPortType, bool isVisionSensorSubscriber);
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


void SubscriberRTC::createInPorts(BodyItem* bodyItem, int pointCloudPortType, bool isVisionSensorSubscriber)
{
    if(!isVisionSensorSubscriber){
        inputs.push_back(new KinematicStateInput(this, bodyItem));
    }
    
    DeviceList<> devices = bodyItem->body()->devices();
    
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


KinematicStateInput::KinematicStateInput(SubscriberRTC* rtc, BodyItem* bodyItem)
    : bodyItem(bodyItem),
      qin("q", q),
      updateKinematicStateCaller([&](){ updateKinematicState(); })
{
    rtc->addInPort("q", qin);
}


void KinematicStateInput::read()
{
    if(qin.isNew()){
        do {
            qin.read();
        } while(qin.isNew());

        if(q.data.length() > 0){
            lock_guard<mutex> lock(kinematicStateMutex);
            qtmp.resize(q.data.length());
            for(size_t i=0; i < qtmp.size(); ++i){
                qtmp[i] = q.data[i];
            }
            updateKinematicStateCaller();
        }
    }
}


void KinematicStateInput::updateKinematicState()
{
    auto body = bodyItem->body();
    {
        lock_guard<mutex> lock(kinematicStateMutex);
        int n = std::min(body->numJoints(), (int)qtmp.size());
        for(int i=0; i < n; ++i){
            body->joint(i)->q() = qtmp[i];
        }
    }
    bodyItem->notifyKinematicStateChangeLater(true);
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
    if(pointCloudPortType == BodyStateSubscriberRTCItem::POINT_CLOUD_TYPES_POINT_CLOUD_TYPE){

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
        
    } else if(pointCloudPortType == BodyStateSubscriberRTCItem::RTC_POINT_CLOUD_TYPE){
        rtc->addInPort(port2.name(), port2);
    }
}


void PointCloudInput::read()
{
    if(pointCloudPortType == BodyStateSubscriberRTCItem::POINT_CLOUD_TYPES_POINT_CLOUD_TYPE){
        read1();
    } else if(pointCloudPortType == BodyStateSubscriberRTCItem::RTC_POINT_CLOUD_TYPE){
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

class BodyStateSubscriberRTCItemImpl
{
public:
    BodyStateSubscriberRTCItem* self;
    MessageView* mv;
    BodyItem* bodyItem;
    SubscriberRTC* subscriberRTC;
    RTC::ExecutionContext_var execContext;
    int periodicRate;
    Selection pointCloudPortType;

    BodyStateSubscriberRTCItemImpl(BodyStateSubscriberRTCItem* self);
    BodyStateSubscriberRTCItemImpl(BodyStateSubscriberRTCItem* self, const BodyStateSubscriberRTCItemImpl& org);
    ~BodyStateSubscriberRTCItemImpl();

    void setBodyItem(BodyItem* bodyItem);
    void createRTC();
    void deleteRTC();
    bool start();
    void stop();
};

}


void BodyStateSubscriberRTCItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<BodyStateSubscriberRTCItem>(N_("BodyStateSubscriberRTCItem"));
    ext->itemManager().registerClass<VisionSensorSubscriberRTCItem>(N_("VisionSensorSubscriberRTCItem"));
    ext->itemManager().addCreationPanel<BodyStateSubscriberRTCItem>();

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


BodyStateSubscriberRTCItem::BodyStateSubscriberRTCItem()
{
    impl = new BodyStateSubscriberRTCItemImpl(this);
}


BodyStateSubscriberRTCItem::BodyStateSubscriberRTCItem(const BodyStateSubscriberRTCItem& org)
    : Item(org)
{
    impl = new BodyStateSubscriberRTCItemImpl(this);
}


BodyStateSubscriberRTCItemImpl::BodyStateSubscriberRTCItemImpl(BodyStateSubscriberRTCItem* self)
    : self(self),
      mv(MessageView::instance()),
      pointCloudPortType(BodyStateSubscriberRTCItem::N_POINT_CLOUD_PORT_TYPES, CNOID_GETTEXT_DOMAIN_NAME)
{
    bodyItem = nullptr;
    subscriberRTC = nullptr;
    execContext = RTC::ExecutionContext::_nil();
    periodicRate = 30;

    pointCloudPortType.setSymbol(
        BodyStateSubscriberRTCItem::POINT_CLOUD_TYPES_POINT_CLOUD_TYPE,
        "PointCloudTypes::PointCloud");
    pointCloudPortType.setSymbol(
        BodyStateSubscriberRTCItem::RTC_POINT_CLOUD_TYPE,
        "RTC::PointCloud");
    pointCloudPortType.select(BodyStateSubscriberRTCItem::POINT_CLOUD_TYPES_POINT_CLOUD_TYPE);
    
    self->sigNameChanged().connect([&](const string&){ createRTC(); });
}


BodyStateSubscriberRTCItemImpl::BodyStateSubscriberRTCItemImpl
(BodyStateSubscriberRTCItem* self, const BodyStateSubscriberRTCItemImpl& org)
    : BodyStateSubscriberRTCItemImpl(self)
{
    periodicRate = org.periodicRate;
    pointCloudPortType = org.pointCloudPortType;
}


BodyStateSubscriberRTCItem::~BodyStateSubscriberRTCItem()
{
    delete impl;
}


BodyStateSubscriberRTCItemImpl::~BodyStateSubscriberRTCItemImpl()
{
    deleteRTC();
}


void BodyStateSubscriberRTCItem::onDisconnectedFromRoot()
{
    impl->deleteRTC();
}


void BodyStateSubscriberRTCItem::onPositionChanged()
{
    impl->setBodyItem(findOwnerItem<BodyItem>());
}


void BodyStateSubscriberRTCItemImpl::setBodyItem(BodyItem* newBodyItem)
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


void BodyStateSubscriberRTCItem::setPeriodicRate(int rate)
{
    if(rate != impl->periodicRate){
        impl->periodicRate = rate;
        impl->createRTC();
    }
}


void BodyStateSubscriberRTCItem::setPointCloudPortType(int type)
{
    if(type != impl->pointCloudPortType.which()){
        impl->pointCloudPortType.select(type);
        impl->createRTC();
    }
}


void BodyStateSubscriberRTCItemImpl::createRTC()
{
    deleteRTC();

    if(!bodyItem){
        return;
    }
    
    const string param(
        "VisionSensorSubscriber?"
        "instance_name={0}&"
        "exec_cxt.periodic.type=PeriodicExecutionContext&"
        "exec_cxt.periodic.rate={1}");
    
    RTC::RtcBase* rtc = createManagedRTC(format(param, self->name(), periodicRate));
    if(!rtc){
        mv->putln(format(_("RTC for \"{}\" cannot be created."), self->name()), MessageView::ERROR);
        return;
    }

    subscriberRTC = dynamic_cast<SubscriberRTC*>(rtc);

    // For the backward compatibility
    bool isVisionSensorSubscriber = dynamic_cast<VisionSensorSubscriberRTCItem*>(self);
    
    subscriberRTC->createInPorts(bodyItem, pointCloudPortType.which(), isVisionSensorSubscriber);

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


void BodyStateSubscriberRTCItemImpl::deleteRTC()
{
    if(subscriberRTC){
        subscriberRTC->exit();
        RTC::Manager::instance().cleanupComponents();
        subscriberRTC = nullptr;
    }
}


Item* BodyStateSubscriberRTCItem::doDuplicate() const
{
    return new BodyStateSubscriberRTCItem(*this);
}


void BodyStateSubscriberRTCItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty.decimals(3)(_("Periodic rate"), impl->periodicRate,
                            [&](int rate){ setPeriodicRate(rate); return true; });

    putProperty(_("Point cloud port type"), impl->pointCloudPortType,
                [&](int which){ setPointCloudPortType(which); return true; });
}


bool BodyStateSubscriberRTCItem::store(Archive& archive)
{
    archive.write("periodicRate", impl->periodicRate);
    archive.write("pointCloudPortType", impl->pointCloudPortType.selectedSymbol(), DOUBLE_QUOTED);
    return true;
}


bool BodyStateSubscriberRTCItem::restore(const Archive& archive)
{
    archive.read("periodicRate", impl->periodicRate);

    string type;
    if(archive.read("pointCloudPortType", type)){
        impl->pointCloudPortType.select(type);
    }
    
    return true;
}
