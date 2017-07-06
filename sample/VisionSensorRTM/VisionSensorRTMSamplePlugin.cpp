/**
   @author Shizuko Hattori
*/

#include <deque>
#include <cnoid/Plugin>
#include <cnoid/ViewManager>
#include <cnoid/ImageView>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/MessageView>
#include <cnoid/Camera>
#include <cnoid/RangeSensor>
#include <cnoid/Config>
#include <cnoid/PointSetItem>
#include <cnoid/corba/CameraImage.hh>
#include <cnoid/corba/PointCloud.hh>
#include <cnoid/LazyCaller>
#include <cnoid/OpenRTMUtil>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/InterfaceDataTypes.hh>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <mutex>

#ifdef _WIN32
#include <Windows.h>
#endif
#include <GL/glew.h>

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using namespace RTC;
using boost::format;

const int BUF_SIZE = 200;

template <class T> double toSec(T t)
{
    return t.sec + t.nsec / 1000000000.0;
}

class VisionSensorSampleRTC : public DataFlowComponentBase
{
public :
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Img::TimedCameraImage timedCameraImage;
    PointCloudTypes::PointCloud timedPointCloud;
    RTC::RangeData timedRangeData;
    InPort<Img::TimedCameraImage> imageInPort;
    InPort<PointCloudTypes::PointCloud> pointCloudInPort;
    InPort<RTC::RangeData> rangeDataInPort;
    TimedDoubleSeq timedCamera_T;
    InPort<TimedDoubleSeq> camera_TInPort;

    std::mutex mtx;
    Image image;
    ImageView* imageView;
    vector<Vector3f> rangeCameraPoints;
    vector<Vector3f> rangeCameraColors;
    vector<Vector3f> rangeSensorPoints;
    Position camera_T;
    Position camera_local_T;
    Position rangeSensor_local_T;
    SgPointSet* pointSetFromRangeCamera;
    SgPointSet* pointSetFromRangeSensor;

    struct TimedT {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double time;
        Position T;
    };
    deque<TimedT, Eigen::aligned_allocator<TimedT> > timedTbuf;

    void setImageView(ImageView* _imageView)
    {
        imageView = _imageView;
    }

    void setCameraLocalT(const Position& T_local)
    {
        camera_local_T = T_local;
    }

    void setRangeSensorLocalT(const Position& T_local)
    {
        rangeSensor_local_T = T_local;
    }

    void setPointSetFromRangeCamera(SgPointSet* _pointSetFromRangeCamera)
    {
        pointSetFromRangeCamera = _pointSetFromRangeCamera;
    }

    void setPointSetFromRangeSensor(SgPointSet* _pointSetFromRangeSensor)
    {
        pointSetFromRangeSensor = _pointSetFromRangeSensor;
    }

    static void registerFactory(Manager* manager, const char* componentTypeName)
    {
        static const char* spec[] = {
            "implementation_id", "VisionSensorSample",
            "type_name",         "VisionSensorSample",
            "description",       "This component is Vision Sensoe Sample.",
            "version",           CNOID_VERSION_STRING,
            "vendor",            "AIST",
            "category",          "Choreonoid",
            "activity_type",     "DataFlowComponent",
            "max_instance",      "100",
            "language",          "C++",
            "lang_type",         "compile",
            ""
        };

        Properties profile(spec);
        profile.setDefault("type_name", componentTypeName);

        manager->registerFactory(profile,
                                 Create<VisionSensorSampleRTC>,
                                 Delete<VisionSensorSampleRTC>);
    };

    VisionSensorSampleRTC(Manager* manager)
        : DataFlowComponentBase(manager),
          imageInPort("cameraImage", timedCameraImage),
          pointCloudInPort("cameraRange", timedPointCloud),
          rangeDataInPort("rangeSensor", timedRangeData),
          camera_TInPort("cameraTrans", timedCamera_T) {};

    virtual ~VisionSensorSampleRTC(){ };

    ReturnCode_t onInitialize()
    {
        addInPort("cameraImage", imageInPort);
        addInPort("cameraRange", pointCloudInPort);
        addInPort("rangeSensor", rangeDataInPort);
        addInPort("cameraTrans", camera_TInPort);
        
        camera_T.setIdentity();
        imageView = 0;
        pointSetFromRangeCamera = 0;
        pointSetFromRangeSensor = 0;
        
        return RTC_OK;
    }

    ReturnCode_t onActivated(UniqueId ec_id)
    {
        timedTbuf.clear();
        timedTbuf.resize(BUF_SIZE);

        return RTC_OK;
    }

    ReturnCode_t onDeactivated(UniqueId ec_id)
    {
        callLater(bind(&VisionSensorSampleRTC::clearImage,this));
        return RTC_OK;
    }

    void detectCamera_T(double time)
    {
        for(deque<TimedT, Eigen::aligned_allocator<TimedT> >::reverse_iterator it = timedTbuf.rbegin();
            it != timedTbuf.rend(); it++){
            if(time >= it->time){
                camera_T = it->T;
                return;;
            }
        }
        camera_T = timedTbuf.front().T;
    }

    ReturnCode_t onExecute(UniqueId ec_id)
    {
        if(camera_TInPort.isNew()){
            do {
                camera_TInPort.read();
                TimedT timedT;
                timedT.time = toSec(timedCamera_T.tm);
                timedT.T(0,3) = timedCamera_T.data[0];
                timedT.T(1,3) = timedCamera_T.data[1];
                timedT.T(2,3) = timedCamera_T.data[2];
                timedT.T(0,0) = timedCamera_T.data[3];
                timedT.T(0,1) = timedCamera_T.data[4];
                timedT.T(0,2) = timedCamera_T.data[5];
                timedT.T(1,0) = timedCamera_T.data[6];
                timedT.T(1,1) = timedCamera_T.data[7];
                timedT.T(1,2) = timedCamera_T.data[8];
                timedT.T(2,0) = timedCamera_T.data[9];
                timedT.T(2,1) = timedCamera_T.data[10];
                timedT.T(2,2) = timedCamera_T.data[11];
                timedTbuf.push_back(timedT);
                if(timedTbuf.size() > BUF_SIZE)
                    timedTbuf.pop_front();
            }while(camera_TInPort.isNew());
        }
        
        if(imageInPort.isNew()){
            do {
                imageInPort.read();
            }while(imageInPort.isNew());
            
            std::unique_lock<std::mutex> lock(mtx);
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
            image.setSize(timedCameraImage.data.image.width, timedCameraImage.data.image.height, numComponents);
            size_t length = timedCameraImage.data.image.raw_data.length();
            memcpy(image.pixels(), timedCameraImage.data.image.raw_data.get_buffer(), length);
            
            callLater(bind(&VisionSensorSampleRTC::updateImage,this));
        }
        
        if(pointCloudInPort.isNew()){
            do {
                pointCloudInPort.read();
            }while(pointCloudInPort.isNew());
            
            detectCamera_T(toSec(timedPointCloud.tm));
            const Affine3f C = (camera_T * camera_local_T).cast<float>();
            
            int numPoints = timedPointCloud.height * timedPointCloud.width;
            bool rgb = false;
            if(timedPointCloud.fields.length() == 6)
                rgb = true;
            
            unsigned char* src = (unsigned char*)timedPointCloud.data.get_buffer();
            std::unique_lock<std::mutex> lock(mtx);
            rangeCameraPoints.resize(numPoints);
            if(rgb)
                rangeCameraColors.resize(numPoints);
            else
                rangeCameraColors.clear();
            Vector3f point;
            for(size_t i=0; i<numPoints; i++, src+=timedPointCloud.point_step){
                memcpy(&point.x(), &src[0], 4);
                memcpy(&point.y(), &src[4], 4);
                memcpy(&point.z(), &src[8], 4);
                rangeCameraPoints[i] = C * point;
                if(rgb){
                    rangeCameraColors[i].x() = src[12] / 255.0;
                    rangeCameraColors[i].y() = src[13] / 255.0;
                    rangeCameraColors[i].z() = src[14] / 255.0;
                }
            }
            lock.unlock();
            
            callLater(bind(&VisionSensorSampleRTC::updatePointCloudFromRangeCamera,this));
        }
        
        if(rangeDataInPort.isNew()){
            do {
                rangeDataInPort.read();
            }while(rangeDataInPort.isNew());
            
            detectCamera_T(toSec(timedRangeData.tm));
            
            const Affine3f C = (camera_T * rangeSensor_local_T).cast<float>();
            int numPoints = timedRangeData.ranges.length();
            double* src = timedRangeData.ranges.get_buffer();
            const double yawStep = timedRangeData.config.angularRes;
            const double yawMin = timedRangeData.config.minAngle;
            const double maxDistance = timedRangeData.config.maxRange;
            
            std::unique_lock<std::mutex> lock(mtx);
            rangeSensorPoints.clear();
            rangeSensorPoints.reserve(numPoints);
            for(int yaw=0; yaw < numPoints; yaw++){
                const double distance = src[yaw];
                if(distance <= maxDistance){
                    double yawAngle = yawMin + yaw * yawStep;
                    Vector3f point;
                    point.x() = distance * sin(-yawAngle);
                    point.y() = 0.0;
                    point.z() = -distance * cos(yawAngle);
                    rangeSensorPoints.push_back(C * point);
                }
            }
            lock.unlock();
            
            callLater(bind(&VisionSensorSampleRTC::updatePointCloudFromRangeSensor,this));
        }
        
        return RTC::RTC_OK;
    }
    
    void updateImage()
    {
        std::lock_guard<std::mutex> lock(mtx);
        if(image.height() > 1){
            imageView->setImage(image);
        }
    }

    void updatePointCloudFromRangeCamera()
    {
        if(!pointSetFromRangeCamera)
            return;
        
        SgVertexArray& disPoints = *pointSetFromRangeCamera->getOrCreateVertices();
        SgColorArray& disColors = *pointSetFromRangeCamera->getOrCreateColors();
        
        std::unique_lock<std::mutex> lock(mtx);
        disPoints.resize(rangeCameraPoints.size());
        disColors.resize(rangeCameraColors.size());
        copy(rangeCameraPoints.begin(), rangeCameraPoints.end(), disPoints.begin());
        copy(rangeCameraColors.begin(), rangeCameraColors.end(), disColors.begin());
        lock.unlock();
        
        pointSetFromRangeCamera->notifyUpdate();
    }

    void updatePointCloudFromRangeSensor()
    {
        if(!pointSetFromRangeSensor)
            return;
        
        SgVertexArray& disPoints = *pointSetFromRangeSensor->getOrCreateVertices();
        
        std::unique_lock<std::mutex> lock(mtx);
        disPoints.resize(rangeSensorPoints.size());
        copy(rangeSensorPoints.begin(), rangeSensorPoints.end(), disPoints.begin());
        lock.unlock();
        
        pointSetFromRangeSensor->notifyUpdate();
    }
    
    void clearImage()
    {
        if(imageView){
            image.clear();
            imageView->setImage(image);
        }
        
        if(pointSetFromRangeCamera){
            pointSetFromRangeCamera->getOrCreateVertices()->clear();
            pointSetFromRangeCamera->getOrCreateColors()->clear();
        }
        
        if(pointSetFromRangeSensor){
            pointSetFromRangeSensor->getOrCreateVertices()->clear();
            pointSetFromRangeSensor->getOrCreateColors()->clear();
        }
    }
};


class VisionSensorRTMSamplePlugin : public Plugin
{
    ImageView* imageView;
    Connection sigItemAddedConnection;
    VisionSensorSampleRTC* visionSensorSampleRTC;
    CameraPtr camera;
    RangeSensorPtr rangeSensor;
    SgPointSetPtr pointSetFromRangeCamera;
    SgPointSetPtr pointSetFromRangeSensor;

public:

    VisionSensorRTMSamplePlugin() : Plugin("VisionSensorRTMSample")
    {
        require("Body");
        require("OpenRTM");
    }
    
    virtual bool initialize()
    {
        imageView = ViewManager::getOrCreateView<ImageView>("CameraImage", true);
        imageView->setScalingEnabled(true);

        Manager& rtcManager = Manager::instance();
        VisionSensorSampleRTC::registerFactory(&rtcManager, "VisionSensorSample");
        const char* param = "VisionSensorSample?instance_name=VisionSensorSample&exec_cxt.periodic.type=PeriodicExecutionContext&exec_cxt.periodic.rate=30";
        RTObject_impl* rtc = rtcManager.createComponent(param);
        visionSensorSampleRTC = dynamic_cast<VisionSensorSampleRTC*>(rtc);
        visionSensorSampleRTC->setImageView(imageView);
        
        camera =  0;
        rangeSensor = 0;
        sigItemAddedConnection =
            RootItem::instance()->sigItemAdded().connect(
                std::bind(&VisionSensorRTMSamplePlugin::onItemAdded, this, _1));
        
        return true;
    }

    virtual bool finalize()
    {
        deleteRTC(visionSensorSampleRTC);
        return true;
    }

    void onItemAdded(Item* item)
    {
        MessageView* mv = MessageView::instance();
        
        if(BodyItem* bodyItem = dynamic_cast<BodyItem*>(item)){
            Body* body = bodyItem->body();
            for(size_t i=0; i < body->numDevices(); ++i){
                Device* device = body->device(i);
                if(!camera){
                    camera = dynamic_cast<Camera*>(device);
                    if(camera){
                        mv->putln(format("VisionSensorRTMSamplePlugin: Detected Camera \"%1%\" of %2%.")
                                  % camera->name() % body->name());
                        visionSensorSampleRTC->setCameraLocalT(camera->T_local());
                    }
                }
                if(!rangeSensor){
                    rangeSensor = dynamic_cast<RangeSensor*>(device);
                    if(rangeSensor){
                        mv->putln(format("VisionSensorRTMSamplePlugin: Detected RangeSensor \"%1%\" of %2%.")
                                  % rangeSensor->name() % body->name());
                        visionSensorSampleRTC->setRangeSensorLocalT(rangeSensor->T_local());
                    }
                }
            }
        }else if(PointSetItem* pointSetItem = dynamic_cast<PointSetItem*>(item)){
            if(pointSetItem->name() == "RangeCameraPoints"){
                pointSetFromRangeCamera = pointSetItem->pointSet();
                mv->putln("VisionSensorRTMSamplePlugin: Detected PointSetItem \"RangeCameraPoints\"");
                visionSensorSampleRTC->setPointSetFromRangeCamera(pointSetFromRangeCamera);
            } else if(pointSetItem->name() == "RangeSensorPoints"){
                pointSetFromRangeSensor = pointSetItem->pointSet();
                mv->putln("VisionSensorRTMSamplePlugin: Detected PointSetItem \"RangeSensorPoints\"");
                visionSensorSampleRTC->setPointSetFromRangeSensor(pointSetFromRangeSensor);
            }
        }
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(VisionSensorRTMSamplePlugin);
