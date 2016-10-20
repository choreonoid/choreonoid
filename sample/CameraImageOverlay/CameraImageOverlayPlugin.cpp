/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/SceneView>
#include <cnoid/SceneShape>
#include <cnoid/Timer>
#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace cnoid;

namespace {

class TextureOverlay : public SgOverlay
{
public:
    SgTexturePtr texture;
        
    TextureOverlay() {

        SgShape* shape = new SgShape;
        SgMesh* mesh = shape->setMesh(new SgMesh);
        
        SgVertexArray& vertices = *mesh->setVertices(new SgVertexArray);
        vertices.resize(4);
        static const float z = 1.0f - std::numeric_limits<float>::epsilon();
        vertices[0] <<  1.0,  1.0, z;
        vertices[1] <<  1.0, -1.0, z;
        vertices[2] << -1.0, -1.0, z;
        vertices[3] << -1.0,  1.0, z;
        
        texture = shape->setTexture(new SgTexture);
        
        SgTexCoordArray& texCoords = *mesh->setTexCoords(new SgTexCoordArray);
        texCoords.resize(4);
        texCoords[0] << 1.0, 0.0;
        texCoords[1] << 1.0, 1.0;
        texCoords[2] << 0.0, 1.0;
        texCoords[3] << 0.0, 0.0;
        
        mesh->setNumTriangles(2);
        mesh->setTriangle(0, 0, 2, 1);
        mesh->setTriangle(1, 0, 3, 2);
        mesh->texCoordIndices() = mesh->triangleVertices();
        
        addChild(shape);
    }

    virtual void calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume) {

        SgImage* image = texture->getOrCreateImage();
        double tw = image->width();
        double th = image->height();
        if(tw > 0 && th > 0){
            if(tw >= th){
                const double r = (viewportHeight / viewportWidth) * (tw / th);
                io_volume.left = -1.0;
                io_volume.right = 1.0;
                io_volume.bottom = -r;
                io_volume.top = r;
            } else {
                const double r = (viewportWidth / viewportHeight) * (th / tw);
                io_volume.left = -r;
                io_volume.right = r;
                io_volume.bottom = -1.0;
                io_volume.top = 1.0;
            }
        }
    }
};

typedef cnoid::ref_ptr<TextureOverlay> TextureOverlayPtr;

}


class CameraImageOverlayPlugin : public Plugin
{
public:

    TextureOverlayPtr overlay;
    CvCapture* capture;
    Timer timer;
    
    CameraImageOverlayPlugin() : Plugin("CameraImageOverlay") {
        capture = 0;
    }
    
    virtual bool initialize() {
        
        capture = cvCreateCameraCapture(0);
        //cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
        //cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);
        
        overlay = new TextureOverlay;
        
        SceneView::instance()->scene()->addChild(overlay, true);
        
        timer.sigTimeout().connect(bind(&CameraImageOverlayPlugin::updateCameraImage, this));
        timer.start(33);  //msec
        
        return true;
    }

    void updateCameraImage() {

        if(!capture)
            return;

        IplImage* frame = cvQueryFrame(capture);

        const int width = frame->width;
        const int height = frame->height;
        SgImage* image = overlay->texture->getOrCreateImage();
        image->setSize(width, height, frame->nChannels);
        
        unsigned char* dest = image->pixels();
        for(int i=0; i < height; ++i){
            const int y = i * width * 3;
            for(int j=0; j < width; ++j){
                const int x = y + j * 3;
                dest[x] = frame->imageData[x + 2];
                dest[x + 1] = frame->imageData[x + 1];
                dest[x + 2] = frame->imageData[x];
            }
        }
        
        overlay->texture->image()->notifyUpdate();
    }

    virtual bool finalize() {

        SceneView::instance()->scene()->removeChild(overlay, true);

        timer.stop();
        
        if(capture){
            cvReleaseCapture(&capture);
        }
        return true;
    }
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(CameraImageOverlayPlugin)
