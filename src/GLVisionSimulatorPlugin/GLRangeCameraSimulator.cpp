#include "GLRangeCameraSimulator.h"
#include "GLVisionSimulatorItem.h"
#include <cnoid/GLSceneRenderer>

using namespace std;
using namespace cnoid;


GLRangeCameraSimulator::GLRangeCameraSimulator(RangeCamera* rangeCamera)
    : GLCameraSimulator(rangeCamera),
      rangeCamera(rangeCamera)
{

}


bool GLRangeCameraSimulator::doInitialize(GLVisionSimulatorItem* visionSimulatorItem)
{
    if(camera->lensType() != Camera::NORMAL_LENS){
        return false;
    }
    imageType = rangeCamera->imageType();
    if(imageType != Camera::NO_IMAGE && imageType != Camera::COLOR_IMAGE){
        return false;
    }
    
    double frameRate = std::max(0.1, std::min(camera->frameRate(), visionSimulatorItem->maxFrameRate()));
    setCycleTime(1.0 / frameRate);

    if(visionSimulatorItem->isVisionDataRecordingEnabled()){
        camera->setImageStateClonable(true);
    }

    hasRo = !rangeCamera->opticalFrameRotation().isIdentity();
    if(hasRo){
        Ro = rangeCamera->opticalFrameRotation().cast<float>();
    }
    isOrganized = rangeCamera->isOrganized();
    detectionRate = rangeCamera->detectionRate();
    errorDeviation = rangeCamera->errorDeviation();

    if(rangeCamera->errorDeviation() > 0.0){
        distanceErrorDistribution.param(
            std::normal_distribution<>::param_type(0.0, rangeCamera->errorDeviation()));
    }

    randomNumber.seed(0);
    detectionProbability.reset();
    distanceErrorDistribution.reset();

    return true;
}


bool GLRangeCameraSimulator::doInitializeScreenCamera(GLVisionSensorRenderingScreen* screen)
{
    // Enable depth buffer update for depth data acquisition
    screen->setDepthBufferUpdateEnabled(true);
    return screen->useCameraDeviceAsScreenCamera();
}

        
void GLRangeCameraSimulator::doStoreScreenImage(GLVisionSensorRenderingScreen* screen)
{
    if(!image){
        image = std::make_shared<Image>();
    }

    points = std::make_shared<vector<Vector3f>>();

    unsigned char* pixels = nullptr;
    int resolutionX = screen->resolutionX();
    int resolutionY = screen->resolutionY();

    const bool extractColors = (imageType == Camera::COLOR_IMAGE);
    if(extractColors){
        colorBuf.resize(resolutionX * resolutionY * 3 * sizeof(unsigned char));
        screen->readImageBuffer(&colorBuf[0]);
        if(isOrganized){
            image->setSize(resolutionX, resolutionY, 3);
        } else {
            image->setSize(resolutionX * resolutionY, 1, 3);
        }
        pixels = image->pixels();
    }

    depthBuf.resize(resolutionX * resolutionY * sizeof(float));
    screen->readDepthBuffer(depthBuf);

    const Matrix4f Pinv = screen->renderer()->projectionMatrix().inverse().cast<float>();
    const float fw = resolutionX;
    const float fh = resolutionY;
    const int cx = resolutionX / 2;
    const int cy = resolutionY / 2;
    Vector4f n;
    n[3] = 1.0f;
    points->clear();
    points->reserve(resolutionX * resolutionY);
    unsigned char* colorSrcTop;
    unsigned char* colorSrc;

    isDense = true;
    
    for(int y = resolutionY - 1; y >= 0; --y){
        int srcpos = y * resolutionX;
        if(extractColors){
            colorSrcTop = &colorBuf[0] + y * resolutionX * 3;
        }
        for(int x=0; x < resolutionX; ++x){
            float z = depthBuf[srcpos + x];

            if(detectionRate < 1.0){
                if(detectionProbability(randomNumber) > detectionRate){
                    if(!isOrganized){
                        continue;
                    } else {
                        z = 1.0f;
                    }
                }
            }

            if(z > 0.0f && z < 1.0f){
                n.x() = 2.0f * x / fw - 1.0f;
                n.y() = 2.0f * y / fh - 1.0f;
                n.z() = 2.0f * z - 1.0f;
                const Vector4f o = Pinv * n;
                const float& w = o[3];
                Vector3f p(o[0] / w, o[1] / w, o[2] / w);

                if(errorDeviation > 0.0){
                    double d = p.norm();
                    double r = (d + distanceErrorDistribution(randomNumber)) / d;
                    p *= r;
                }

                if(hasRo){
                    points->push_back(Ro * p);
                } else {
                    points->push_back(p);
                }
                if(pixels){
                    colorSrc = colorSrcTop + x * 3;
                    pixels[0] = colorSrc[0];
                    pixels[1] = colorSrc[1];
                    pixels[2] = colorSrc[2];
                    pixels += 3;
                }
            } else if(isOrganized){
                Vector3f p;
                if(z <= 0.0f){
                    p.z() = numeric_limits<float>::infinity();
                } else {
                    p.z() = -numeric_limits<float>::infinity();
                }
                if(x == cx){
                    p.x() = 0.0;
                } else {
                    p.x() = (x - cx) * numeric_limits<float>::infinity();
                }
                if(y == cy){
                    p.y() = 0.0;
                } else {
                    p.y() = (y - cy) * numeric_limits<float>::infinity();
                }
                if(pixels){
                    colorSrc = colorSrcTop + x * 3;
                    pixels[0] = colorSrc[0];
                    pixels[1] = colorSrc[1];
                    pixels[2] = colorSrc[2];
                    pixels += 3;
                }
                isDense = false;
                if(hasRo){
                    points->push_back(Ro * p);
                } else {
                    points->push_back(p);
                }
            }
        }
    }

    if(extractColors && !isOrganized){
        image->setSize((pixels - image->pixels()) / 3, 1, 3);
    }
}


void GLRangeCameraSimulator::doClearVisionSensorData()
{
    GLCameraSimulator::doClearVisionSensorData();
    rangeCamera->clearPoints();
}


void GLRangeCameraSimulator::doUpdateVisionSensorData()
{
    GLCameraSimulator::doUpdateVisionSensorData();

    rangeCamera->setPoints(points);
    rangeCamera->setDense(isDense);
}
