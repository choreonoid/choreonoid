/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/ViewManager>
#include <cnoid/ImageView>
#include <cnoid/SceneView>
#include <cnoid/MessageView>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/PointSetItem>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <iostream>

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using boost::format;

class VisionSensorSamplePlugin : public Plugin
{
    ImageView* imageView;
    Connection sigItemAddedConnection;
    CameraPtr camera;
    std::shared_ptr<const Image> prevImage;
    RangeCameraPtr rangeCamera;
    SgPointSetPtr pointSetFromRangeCamera;
    std::shared_ptr<const std::vector<Vector3f> > prevPoints;
    RangeSensorPtr rangeSensor;
    SgPointSetPtr pointSetFromRangeSensor;
    std::shared_ptr<const RangeSensor::RangeData> prevRangeData;
    
public:
    
    VisionSensorSamplePlugin() : Plugin("VisionSensorSample") {
        require("Body");
    }
    
    virtual bool initialize() {
        imageView = ViewManager::getOrCreateView<ImageView>("CameraImage", true);
        imageView->setScalingEnabled(true);
        
        sigItemAddedConnection =
            RootItem::instance()->sigItemAdded().connect(
                bind(&VisionSensorSamplePlugin::onItemAdded, this, _1));
        
        return true;
    }

    void onItemAdded(Item* item) {
        MessageView* mv = MessageView::instance();
        
        if(BodyItem* bodyItem = dynamic_cast<BodyItem*>(item)){
            Body* body = bodyItem->body();
            for(size_t i=0; i < body->numDevices(); ++i){
                Device* device = body->device(i);
                if(!camera){
                    camera = dynamic_cast<Camera*>(device);
                    if(camera){
                        mv->putln(format("CameraImageView: Detected Camera \"%1%\" of %2%.")
                                  % camera->name() % body->name());
                        rangeCamera = dynamic_pointer_cast<RangeCamera>(camera);
                        camera->sigStateChanged().connect(
                            bind(&VisionSensorSamplePlugin::onCameraStateChanged, this));
                    }
                }
                if(!rangeSensor){
                    rangeSensor = dynamic_cast<RangeSensor*>(device);
                    if(rangeSensor){
                        mv->putln(format("CameraImageView: Detected RangeSensor \"%1%\" of %2%.")
                                  % rangeSensor->name() % body->name());
                        rangeSensor->sigStateChanged().connect(
                            bind(&VisionSensorSamplePlugin::onRangeSensorStateChanged, this));
                    }
                }
            }
        } else if(PointSetItem* pointSetItem = dynamic_cast<PointSetItem*>(item)){
            if(pointSetItem->name() == "RangeCameraPoints"){
                pointSetFromRangeCamera = pointSetItem->pointSet();
                mv->putln("CameraImageView: Detected PointSetItem \"RangeCameraPoints\".");
            } else if(pointSetItem->name() == "RangeSensorPoints"){
                pointSetFromRangeSensor = pointSetItem->pointSet();
                mv->putln("CameraImageView: Detected PointSetItem \"RangeSensorPoints\"");
            }
        }
    }

    void onCameraStateChanged() {
        if(camera->sharedImage() != prevImage){
            const Image& image = camera->constImage();
            if(image.height() > 1){
                imageView->setImage(image);
            }
            prevImage = camera->sharedImage();
        }
        
        if(rangeCamera && (rangeCamera->sharedPoints() != prevPoints) && pointSetFromRangeCamera){
            updatePointSetFromCamera();
            prevPoints = rangeCamera->sharedPoints();
        }
    }

    void updatePointSetFromCamera() {
        const Affine3f C = (camera->link()->T() * camera->T_local()).cast<float>();
        const vector<Vector3f>& src = rangeCamera->constPoints();
        SgVertexArray& points = *pointSetFromRangeCamera->getOrCreateVertices();
        const int numPoints = src.size();
        points.resize(numPoints);
        for(int i=0; i < numPoints; ++i){
            points[i] = C * src[i];
        }
        
        SgColorArray& colors = *pointSetFromRangeCamera->getOrCreateColors();
        const Image& image = camera->constImage();
        if(image.empty() || image.numComponents() != 3){
            colors.clear();
        } else {
            const unsigned char* pixels = image.pixels();
            const int numPixels = image.width() * image.height();
            const int n = std::min(numPixels, numPoints);
            colors.resize(n);
            for(int i=0; i < n; ++i){
                Vector3f& c = colors[i];
                c[0] = *pixels++ / 255.0;
                c[1] = *pixels++ / 255.0;;
                c[2] = *pixels++ / 255.0;;
            }
        }
        pointSetFromRangeCamera->notifyUpdate();
    }

    void onRangeSensorStateChanged() {
        if(rangeSensor->sharedRangeData() != prevRangeData && pointSetFromRangeSensor){
            const Affine3 C = rangeSensor->link()->T() * rangeSensor->T_local();
            const RangeSensor::RangeData& src = rangeSensor->constRangeData();
            SgVertexArray& points = *pointSetFromRangeSensor->getOrCreateVertices();
            points.clear();
            if(!src.empty()){
                points.reserve(src.size());
                const double pitchStep = rangeSensor->pitchStep();
                const double yawStep = rangeSensor->yawStep();
                for(int pitch=0; pitch < rangeSensor->pitchResolution(); ++pitch){
                    const double pitchAngle = pitch * pitchStep - rangeSensor->pitchRange() / 2.0;
                    const int srctop = pitch * rangeSensor->yawResolution();
                    for(int yaw=0; yaw < rangeSensor->yawResolution(); ++yaw){
                        const double distance = src[srctop + yaw];
                        if(distance <= rangeSensor->maxDistance()){
                            double yawAngle = yaw * yawStep - rangeSensor->yawRange() / 2.0;
                            Vector3 point;
                            point.x() = distance * sin(-yawAngle);
                            point.y() = distance * sin(pitchAngle);
                            point.z() = -distance * cos(pitchAngle) * cos(yawAngle);
                            points.push_back((C * point).cast<float>());
                        }
                    }
                }
            }
            prevRangeData = rangeSensor->sharedRangeData();
            pointSetFromRangeSensor->notifyUpdate();
        }
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(VisionSensorSamplePlugin);
