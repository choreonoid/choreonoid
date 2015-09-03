/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "GLSceneRenderer.h"
#include <cnoid/SceneShape>
#include <cnoid/SceneCamera>
#include <cnoid/SceneLight>
#include <cnoid/EigenUtil>
#include <Eigen/StdVector>
#ifdef _WIN32
#include <Windows.h>
#endif
#include <GL/glew.h>
#ifdef _WIN32
#include <GL/wglew.h>
#endif
#include <boost/unordered_map.hpp>
#include <boost/dynamic_bitset.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <iostream>

//#define DO_DEPTH_BUFFER_TYPE_TEST

#ifdef DO_DEPTH_BUFFER_TYPE_TEST
#include <QElapsedTimer>
QElapsedTimer timer;
#endif

using namespace std;
using namespace cnoid;

namespace {

const bool USE_DISPLAY_LISTS = true;
const bool USE_VBO = false;
const bool USE_INDEXING = false;
const bool SHOW_IMAGE_FOR_PICKING = false;

const float MinLineWidthForPicking = 5.0f;

typedef vector<Affine3, Eigen::aligned_allocator<Affine3> > Affine3Array;

struct TransparentShapeInfo
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SgShape* shape;
    unsigned int pickId;
    Affine3 V; // view matrix
};
typedef boost::shared_ptr<TransparentShapeInfo> TransparentShapeInfoPtr;

    
/*
  A set of variables associated with a scene node
*/
class ShapeCache : public Referenced
{
public:
    GLuint listID;
    GLuint listIDforPicking;
    bool useIDforPicking;
    GLuint bufferNames[4];
    GLuint size;
    vector<TransparentShapeInfoPtr> transparentShapes;
        
    ShapeCache() {
        listID = 0;
        useIDforPicking = false;
        listIDforPicking = 0;
        for(int i=0; i < 4; ++i){
            bufferNames[i] = GL_INVALID_VALUE;
        }
    }
    ~ShapeCache() {
        if(listID){
            glDeleteLists(listID, 1);
        }
        if(listIDforPicking){
            glDeleteLists(listIDforPicking, 1);
        }
        for(int i=0; i < 4; ++i){
            if(bufferNames[i] != GL_INVALID_VALUE){
                glDeleteBuffers(1, &bufferNames[i]);
            }
        }
    }
    GLuint& vertexBufferName() { return bufferNames[0]; }
    GLuint& normalBufferName() { return bufferNames[1]; }
    GLuint& indexBufferName() { return bufferNames[2]; }
    GLuint& texCoordBufferName() { return bufferNames[3]; }
};
typedef ref_ptr<ShapeCache> ShapeCachePtr;


class TextureCache : public Referenced
{
public:
    bool isBound;
    bool isImageUpdateNeeded;
    GLuint textureName;
    int width;
    int height;
    int numComponents;
        
    TextureCache(){
        isBound = false;
        isImageUpdateNeeded = true;
        width = 0;
        height = 0;
        numComponents = 0;
    }
    bool isSameSizeAs(const Image& image){
        return (width == image.width() && height == image.height() && numComponents == image.numComponents());
    }
            
    ~TextureCache(){
        if(isBound){
            glDeleteTextures(1, &textureName);
        }
    }
};
typedef ref_ptr<TextureCache> TextureCachePtr;


struct PreproNode
{
    PreproNode() : parent(0), child(0), next(0) { }
    ~PreproNode() {
        if(child) delete child;
        if(next) delete next;
    }
    enum { GROUP, TRANSFORM, PREPROCESSED, LIGHT, CAMERA };
    boost::variant<SgGroup*, SgTransform*, SgPreprocessed*, SgLight*, SgCamera*> node;
    SgNode* base;
    PreproNode* parent;
    PreproNode* child;
    PreproNode* next;
    template<class T> void setNode(T* n){
        node = n;
        base = n;
    }
};
    
    
class PreproTreeExtractor : public SceneVisitor
{
    PreproNode* node;
    bool found;
public:
    PreproNode* apply(SgNode* node);
    virtual void visitGroup(SgGroup* group);
    virtual void visitTransform(SgTransform* transform);
    virtual void visitShape(SgShape* shape);
    virtual void visitPointSet(SgPointSet* pointSet);        
    virtual void visitLineSet(SgLineSet* lineSet);        
    virtual void visitPreprocessed(SgPreprocessed* preprocessed);
    virtual void visitLight(SgLight* light);
    virtual void visitCamera(SgCamera* camera);
};
}


PreproNode* PreproTreeExtractor::apply(SgNode* snode)
{
    node = 0;
    found = false;
    snode->accept(*this);
    return node;
}


void PreproTreeExtractor::visitGroup(SgGroup* group)
{
    bool foundInSubTree = false;

    PreproNode* self = new PreproNode();
    self->setNode(group);

    for(SgGroup::const_reverse_iterator p = group->rbegin(); p != group->rend(); ++p){

        node = 0;
        found = false;
        
        (*p)->accept(*this);
        
        if(node){
            if(found){
                node->parent = self;
                node->next = self->child;
                self->child = node;
                foundInSubTree = true;
            } else {
                delete node;
            }
        }
    }
            
    found = foundInSubTree;

    if(found){
        node = self;
    } else {
        delete self;
        node = 0;
    }
}


void PreproTreeExtractor::visitTransform(SgTransform* transform)
{
    visitGroup(transform);
    if(node){
        node->setNode(transform);
    }
}


void PreproTreeExtractor::visitShape(SgShape* shape)
{

}


void PreproTreeExtractor::visitPointSet(SgPointSet* shape)
{

}


void PreproTreeExtractor::visitLineSet(SgLineSet* shape)
{

}


void PreproTreeExtractor::visitPreprocessed(SgPreprocessed* preprocessed)
{
    node = new PreproNode();
    node->setNode(preprocessed);
    found = true;
}


void PreproTreeExtractor::visitLight(SgLight* light)
{
    node = new PreproNode();
    node->setNode(light);
    found = true;
}


void PreproTreeExtractor::visitCamera(SgCamera* camera)
{
    node = new PreproNode();
    node->setNode(camera);
    found = true;
}


namespace cnoid {

class GLSceneRendererImpl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    GLSceneRendererImpl(GLSceneRenderer* self, SgGroup* sceneRoot);
    ~GLSceneRendererImpl();

    GLSceneRenderer* self;

    Signal<void()> sigRenderingRequest;

    SgGroupPtr sceneRoot;
    SgGroupPtr scene;

    SgNodePath indexedEntities;

    Affine3Array Vstack; // stack of the model/view matrices

    typedef vector<Vector4f, Eigen::aligned_allocator<Vector4f> > ColorArray;
        
    struct Buf {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        SgVertexArray vertices;
        SgIndexArray triangles;
        SgNormalArray normals;
        ColorArray colors;
        SgTexCoordArray texCoords;
    };
    boost::scoped_ptr<Buf> buf;
        
    struct SgObjectPtrHash {
        std::size_t operator()(const SgObjectPtr& p) const {
            return boost::hash_value<SgObject*>(p.get());
        }
    };
    typedef boost::unordered_map<SgObjectPtr, ReferencedPtr, SgObjectPtrHash> CacheMap;
    CacheMap cacheMaps[2];
    bool doUnusedCacheCheck;
    bool isCheckingUnusedCaches;
    bool hasValidNextCacheMap;
    bool isCacheClearRequested;
    int currentCacheMapIndex;
    CacheMap* currentCacheMap;
    CacheMap* nextCacheMap;
    ShapeCache* currentShapeCache;

    int currentShapeCacheTopViewMatrixIndex;

    bool doPreprocessedNodeTreeExtraction;
    boost::scoped_ptr<PreproNode> preproTree;

    Array4i viewport;

    struct CameraInfo
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CameraInfo() : camera(0), M(Affine3::Identity()), node(0) { }
        CameraInfo(SgCamera* camera, const Affine3& M, PreproNode* node)
            : camera(camera), M(M), node(node) { }
        SgCamera* camera;

        // I want to use 'T' here, but that causes a compile error (c2327) for VC++2010.
        // 'T' is also used as a template parameter in a template code of Eigen,
        // and it seems that the name of a template parameter and this member conflicts.
        // So I changed the name to 'M'.
        // This behavior of VC++ seems stupid!!!
        Affine3 M;
        PreproNode* node;
    };

    typedef vector<CameraInfo, Eigen::aligned_allocator<CameraInfo> > CameraInfoArray;
    CameraInfoArray cameras1;
    CameraInfoArray cameras2;
    CameraInfoArray* cameras;
    CameraInfoArray* prevCameras;

    bool camerasChanged;
    bool currentCameraRemoved;
    int currentCameraIndex;
    SgCamera* currentCamera;
    vector<SgNodePath> cameraPaths;
    Signal<void()> sigCamerasChanged;
    Signal<void()> sigCurrentCameraChanged;
        
    GLfloat aspectRatio; // width / height;

    Affine3 lastViewMatrix;
    Matrix4 lastProjectionMatrix;
    Affine3 tmpCurrentCameraPosition;
    Affine3 tmpCurrentModelTransform;

    struct LightInfo
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        LightInfo() : light(0) { }
        LightInfo(SgLight* light, const Affine3& M) : light(light), M(M) { } 
        SgLight* light;
        Affine3 M;
    };
    vector<LightInfo, Eigen::aligned_allocator<LightInfo> > lights;

    int numSystemLights;
    int prevNumLights;

    Vector3f bgColor;

    SgLightPtr headLight;
    bool isHeadLightLightingFromBackEnabled;
    std::set<SgLightPtr> defaultLights;
    bool additionalLightsEnabled;

    GLSceneRenderer::PolygonMode polygonMode;
    bool defaultLighting;
    bool defaultSmoothShading;
    bool isTextureEnabled;
    SgMaterialPtr defaultMaterial;
    Vector4f defaultColor;
    GLfloat defaultPointSize;
    GLfloat defaultLineWidth;
    GLuint defaultTextureName;
        
    bool doNormalVisualization;
    double normalLength;

    bool isCompiling;
    bool isNewDisplayListDoubleRenderingEnabled;
    bool isNewDisplayListCreated;
    bool isPicking;

    GLdouble pickX;
    GLdouble pickY;
    typedef boost::shared_ptr<SgNodePath> SgNodePathPtr;
    SgNodePath currentNodePath;
    vector<SgNodePathPtr> pickingNodePathList;
    SgNodePath pickedNodePath;
    Vector3 pickedPoint;

    vector<TransparentShapeInfoPtr> transparentShapeInfos;

    // OpenGL states
    enum StateFlag {
        CURRENT_COLOR,
        COLOR_MATERIAL,
        DIFFUSE_COLOR,
        AMBIENT_COLOR,
        EMISSION_COLOR,
        SPECULAR_COLOR,
        SHININESS,
        CULL_FACE,
        CCW,
        LIGHTING,
        LIGHT_MODEL_TWO_SIDE,
        BLEND,
        DEPTH_MASK,
        POINT_SIZE,
        LINE_WIDTH,
        NUM_STATE_FLAGS
    };

    boost::dynamic_bitset<> stateFlag;
        
    Vector4f currentColor;
    Vector4f diffuseColor;
    Vector4f ambientColor;
    Vector4f emissionColor;
    Vector4f specularColor;
    float shininess;
    float lastAlpha;
    bool isColorMaterialEnabled;
    bool isCullFaceEnabled;
    bool isCCW;
    bool isLightingEnabled;
    bool isLightModelTwoSide;
    bool isBlendEnabled;
    bool isDepthMaskEnabled;
    float pointSize;
    float lineWidth;

    void addEntity(SgNode* node);
    void removeEntity(SgNode* node);
    void onSceneGraphUpdated(const SgUpdate& update);
    void updateCameraPaths();
    void setCurrentCamera(int index, bool doRenderingRequest);
    bool setCurrentCamera(SgCamera* camera);
    bool initializeGL();
    void setViewport(int x, int y, int width, int height);
    void beginRendering(bool doRenderingCommands);
    void extractPreproNodes(PreproNode* node, const Affine3& T);
    void renderCamera();
    void getViewVolume(
        const SgOrthographicCamera& camera,
        GLfloat& out_left, GLfloat& out_right, GLfloat& out_bottom, GLfloat& out_top) const;
    void renderLights();
    void renderLight(const SgLight* light, GLint id, const Affine3& T);
    void endRendering();
    void render();
    bool pick(int x, int y);
    inline void setPickColor(unsigned int id);
    inline unsigned int pushPickName(SgNode* node, bool doSetColor = true);
    void popPickName();
    void visitInvariantGroup(SgInvariantGroup* group);
    void visitShape(SgShape* shape);
    void visitPointSet(SgPointSet* pointSet);
    void renderPlot(SgPlot* plot, SgVertexArray& expandedVertices, GLenum primitiveMode);
    void visitLineSet(SgLineSet* lineSet);
    void renderMaterial(const SgMaterial* material);
    bool renderTexture(SgTexture* texture, bool withMaterial);
    void putMeshData(SgMesh* mesh);
    void renderMesh(SgMesh* mesh, bool hasTexture);
    void renderTransparentShapes();
    void writeVertexBuffers(SgMesh* mesh, ShapeCache* cache, bool hasTexture);
    void visitOutlineGroup(SgOutlineGroup* outline);

    void clearGLState();
    void setColor(const Vector4f& color);
    void enableColorMaterial(bool on);
    void setDiffuseColor(const Vector4f& color);
    void setAmbientColor(const Vector4f& color);
    void setEmissionColor(const Vector4f& color);
    void setSpecularColor(const Vector4f& color);
    void setShininess(float shininess);
    void enableCullFace(bool on);
    void setFrontCCW(bool on);
    void enableLighting(bool on);
    void setLightModelTwoSide(bool on);
    void enableBlend(bool on);
    void enableDepthMask(bool on);
    void setPointSize(float size);
    void setLineWidth(float width);
    void getCurrentCameraTransform(Affine3& T);

    Vector4f createColorWithAlpha(const Vector3f& c3){
        Vector4f c4;
        c4.head<3>() = c3;
        c4[3] = lastAlpha;
        return c4;
    }
};
}


GLSceneRenderer::GLSceneRenderer()
{
    SgGroup* sceneRoot = new SgGroup;
    sceneRoot->setName("Root");
    impl = new GLSceneRendererImpl(this, sceneRoot);
}


GLSceneRenderer::GLSceneRenderer(SgGroup* sceneRoot)
{
    impl = new GLSceneRendererImpl(this, sceneRoot);
}


GLSceneRendererImpl::GLSceneRendererImpl(GLSceneRenderer* self, SgGroup* sceneRoot)
    : self(self),
      sceneRoot(sceneRoot)
{
    sceneRoot->sigUpdated().connect(boost::bind(&GLSceneRendererImpl::onSceneGraphUpdated, this, _1));

    Vstack.reserve(16);
    
    buf.reset(new Buf);

    doUnusedCacheCheck = true;
    currentCacheMapIndex = 0;
    hasValidNextCacheMap = false;
    isCacheClearRequested = false;
    currentCacheMap = &cacheMaps[0];
    nextCacheMap = &cacheMaps[1];

    doPreprocessedNodeTreeExtraction = true;

    bgColor << 0.1f, 0.1f, 0.3f; // dark blue

    aspectRatio = 1.0f;

    cameras = &cameras1;
    prevCameras = &cameras2;
    currentCameraIndex = -1;
    currentCamera = 0;

    lastViewMatrix.setIdentity();
    lastProjectionMatrix.setIdentity();

    numSystemLights = 2;
    prevNumLights = 0;

    headLight = new SgDirectionalLight();
    headLight->setAmbientIntensity(0.0f);
    isHeadLightLightingFromBackEnabled = false;
    additionalLightsEnabled = true;

    scene = new SgGroup();
    sceneRoot->addChild(scene);

    polygonMode = GLSceneRenderer::FILL_MODE;
    defaultLighting = true;
    defaultSmoothShading = true;
    defaultMaterial = new SgMaterial;
    defaultColor << 1.0f, 1.0f, 1.0f, 1.0f;
    isTextureEnabled = true;
    defaultPointSize = 1.0f;
    defaultLineWidth = 1.0f;
    
    doNormalVisualization = false;
    normalLength = 0.0;

    isCompiling = false;
    isNewDisplayListDoubleRenderingEnabled = false;
    isNewDisplayListCreated = false;
    isPicking = false;
    pickedPoint.setZero();

    stateFlag.resize(NUM_STATE_FLAGS, false);
    clearGLState();
}


GLSceneRenderer::~GLSceneRenderer()
{
    delete impl;
}


GLSceneRendererImpl::~GLSceneRendererImpl()
{

}


SgGroup* GLSceneRenderer::sceneRoot()
{
    return impl->sceneRoot;
}


SgGroup* GLSceneRenderer::scene()
{
    return impl->scene;
}


void GLSceneRenderer::clearScene()
{
    impl->scene->clearChildren(true);
}


void GLSceneRendererImpl::onSceneGraphUpdated(const SgUpdate& update)
{
    if(update.action() & (SgUpdate::ADDED | SgUpdate::REMOVED)){
        doPreprocessedNodeTreeExtraction = true;
    }
    if(SgImage* image = dynamic_cast<SgImage*>(update.path().front())){
        CacheMap* cacheMap = hasValidNextCacheMap ? nextCacheMap : currentCacheMap;
        CacheMap::iterator p = cacheMap->find(image);
        if(p != cacheMap->end()){
            TextureCache* cache = static_cast<TextureCache*>(p->second.get());
            cache->isImageUpdateNeeded = true;
        }
    }
                
    sigRenderingRequest();
}


SignalProxy<void()> GLSceneRenderer::sigRenderingRequest()
{
    return impl->sigRenderingRequest;
}


SignalProxy<void()> GLSceneRenderer::sigCamerasChanged() const
{
    return impl->sigCamerasChanged;
}


int GLSceneRenderer::numCameras() const
{
    return impl->cameras->size();
}


SgCamera* GLSceneRenderer::camera(int index)
{
    return dynamic_cast<SgCamera*>(cameraPath(index).back());
}


const std::vector<SgNode*>& GLSceneRenderer::cameraPath(int index) const
{
    if(impl->cameraPaths.empty()){
        impl->updateCameraPaths();
    }
    return impl->cameraPaths[index];
}


void GLSceneRendererImpl::updateCameraPaths()
{
    vector<SgNode*> tmpPath;
    const int n = cameras->size();
    cameraPaths.resize(n);
    
    for(int i=0; i < n; ++i){
        CameraInfo& info = (*cameras)[i];
        tmpPath.clear();
        PreproNode* node = info.node;
        while(node){
            tmpPath.push_back(node->base);
            node = node->parent;
        }
        if(!tmpPath.empty()){
            tmpPath.pop_back(); // remove the root node
            vector<SgNode*>& path = cameraPaths[i];
            path.resize(tmpPath.size());
            std::copy(tmpPath.rbegin(), tmpPath.rend(), path.begin());
        }
    }
}


void GLSceneRenderer::setCurrentCamera(int index)
{
    impl->setCurrentCamera(index, true);
}


void GLSceneRendererImpl::setCurrentCamera(int index, bool doRenderingRequest)
{
    SgCamera* newCamera = 0;
    if(index >= 0 && index < cameras->size()){
        newCamera = (*cameras)[index].camera;
    }
    if(newCamera && newCamera != currentCamera){
        currentCameraIndex = index;
        currentCamera = newCamera;
        sigCurrentCameraChanged();
        if(doRenderingRequest){
            sigRenderingRequest();
        }
    }
}


bool GLSceneRenderer::setCurrentCamera(SgCamera* camera)
{
    return impl->setCurrentCamera(camera);
}


bool GLSceneRendererImpl::setCurrentCamera(SgCamera* camera)
{
    if(camera != currentCamera){
        for(size_t i=0; i < cameras->size(); ++i){
            if((*cameras)[i].camera == camera){
                setCurrentCamera(i, true);
                return true;
            }
        }
    }
    return false;
}


SgCamera* GLSceneRenderer::currentCamera() const
{
    return impl->currentCamera;
}


int GLSceneRenderer::currentCameraIndex() const
{
    return impl->currentCameraIndex;
}


SignalProxy<void()> GLSceneRenderer::sigCurrentCameraChanged()
{
    return impl->sigCurrentCameraChanged;
}


bool GLSceneRenderer::setSwapInterval(int interval)
{
#ifdef _WIN32
    return wglSwapIntervalEXT(interval);
#endif
    return false;
}


int GLSceneRenderer::getSwapInterval() const
{
#ifdef _WIN32
    return wglGetSwapIntervalEXT();
#endif
    return -1;
}


bool GLSceneRenderer::initializeGL()
{
    return impl->initializeGL();
}


bool GLSceneRendererImpl::initializeGL()
{
    GLenum err = glewInit();
    if(err != GLEW_OK){
        return false;
    }

    isCullFaceEnabled = false;
    if(isCullFaceEnabled){
        glEnable(GL_CULL_FACE);
        GLint twoSide = 0;
        isLightModelTwoSide = false;
        glLightModeliv(GL_LIGHT_MODEL_TWO_SIDE, &twoSide);
    } else {
        glDisable(GL_CULL_FACE);
        GLint twoSide = 1;
        isLightModelTwoSide = true;
        glLightModeliv(GL_LIGHT_MODEL_TWO_SIDE, &twoSide);
    }
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    setFrontCCW(true);

    glGenTextures(1, &defaultTextureName);

    return true;
}


void GLSceneRenderer::setViewport(int x, int y, int width, int height)
{
    impl->setViewport(x, y, width, height);
}


void GLSceneRendererImpl::setViewport(int x, int y, int width, int height)
{
    aspectRatio = (double)width / height;
    viewport << x, y, width, height;
    glViewport(x, y, width, height);
}


Array4i GLSceneRenderer::viewport() const
{
    return impl->viewport;
}


void GLSceneRenderer::getViewport(int& out_x, int& out_y, int& out_width, int& out_height) const
{
    out_x = impl->viewport[0];
    out_y = impl->viewport[1];
    out_width = impl->viewport[2];
    out_height = impl->viewport[3];
}    


double GLSceneRenderer::aspectRatio() const
{
    return impl->aspectRatio;
}


void GLSceneRenderer::requestToClearCache()
{
    impl->isCacheClearRequested = true;
}


void GLSceneRenderer::initializeRendering()
{
    impl->beginRendering(false);
}


void GLSceneRenderer::beginRendering()
{
    impl->beginRendering(true);
}


void GLSceneRendererImpl::beginRendering(bool doRenderingCommands)
{
    isCheckingUnusedCaches = isPicking ? false : doUnusedCacheCheck;

    if(isCacheClearRequested){
        cacheMaps[0].clear();
        cacheMaps[1].clear();
        hasValidNextCacheMap = false;
        isCheckingUnusedCaches = false;
        isCacheClearRequested = false;
    }
    if(hasValidNextCacheMap){
        currentCacheMapIndex = 1 - currentCacheMapIndex;
        currentCacheMap = &cacheMaps[currentCacheMapIndex];
        nextCacheMap = &cacheMaps[1 - currentCacheMapIndex];
        hasValidNextCacheMap = false;
    }
    currentShapeCache = 0;

    if(doRenderingCommands){
        if(isPicking){
            currentNodePath.clear();
            pickingNodePathList.clear();
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        } else {
            glClearColor(bgColor[0], bgColor[1], bgColor[2], 1.0f);
        }
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    if(doPreprocessedNodeTreeExtraction){
        PreproTreeExtractor extractor;
        preproTree.reset(extractor.apply(sceneRoot));
        doPreprocessedNodeTreeExtraction = false;
    }

    std::swap(cameras, prevCameras);
    cameras->clear();
    camerasChanged = false;
    currentCameraRemoved = true;
    
    lights.clear();

    if(preproTree){
        extractPreproNodes(preproTree.get(), Affine3::Identity());
    }

    if(!camerasChanged){
        if(cameras->size() != prevCameras->size()){
            camerasChanged = true;
        }
    }
    if(camerasChanged){
        if(currentCameraRemoved){
            currentCameraIndex = 0;
        }
        cameraPaths.clear();
        sigCamerasChanged();
    }

    setCurrentCamera(currentCameraIndex, false);

    if(doRenderingCommands && currentCamera){
        renderCamera();
        if(!isPicking){
            renderLights();
        }
        if(isPicking){
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        } else {
            switch(polygonMode){
            case GLSceneRenderer::FILL_MODE:
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                break;
            case GLSceneRenderer::LINE_MODE:
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                break;
            case GLSceneRenderer::POINT_MODE:
                glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
                break;
            }
        }
        if(defaultSmoothShading){
            glShadeModel(GL_SMOOTH);
        } else {
            glShadeModel(GL_FLAT);
        }

        isNewDisplayListCreated = false;
            
        clearGLState();
        
        setColor(defaultColor);
        setPointSize(defaultPointSize);
        //glEnable(GL_POINT_SMOOTH);
        setLineWidth(defaultLineWidth);
        //glEnable(GL_LINE_SMOOTH);

        transparentShapeInfos.clear();
    }
}


void GLSceneRendererImpl::extractPreproNodes(PreproNode* node, const Affine3& T)
{
    switch(node->node.which()){

    case PreproNode::GROUP:
        for(PreproNode* childNode = node->child; childNode; childNode = childNode->next){
            extractPreproNodes(childNode, T);
        }
        break;
        
    case PreproNode::TRANSFORM:
    {
        SgTransform* transform = boost::get<SgTransform*>(node->node);
        Affine3 T1;
        transform->getTransform(T1);
        const Affine3 T2 = T * T1;
        for(PreproNode* childNode = node->child; childNode; childNode = childNode->next){
            extractPreproNodes(childNode, T2);
        }
    }
    break;
        
    case PreproNode::PREPROCESSED:
        // call additional functions
        break;

    case PreproNode::LIGHT:
    {
        SgLight* light = boost::get<SgLight*>(node->node);
        if(additionalLightsEnabled || defaultLights.find(light) != defaultLights.end()){
            lights.push_back(LightInfo(light, T));
        }
        break;
    }

    case PreproNode::CAMERA:
    {
        SgCamera* camera = boost::get<SgCamera*>(node->node);
        size_t index = cameras->size();
        if(!camerasChanged){
            if(index >= prevCameras->size() || camera != (*prevCameras)[index].camera){
                camerasChanged = true;
            }
        }
        if(camera == currentCamera){
            currentCameraRemoved = false;
            currentCameraIndex = cameras->size();
        }
        cameras->push_back(CameraInfo(camera, T, node));
    }
    break;

    default:
        break;
    }
}


void GLSceneRendererImpl::renderCamera()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // set projection
    if(SgPerspectiveCamera* camera = dynamic_cast<SgPerspectiveCamera*>(currentCamera)){
        gluPerspective(degree(camera->fovy(aspectRatio)), aspectRatio, camera->nearDistance(), camera->farDistance());
        
    } else if(SgOrthographicCamera* camera = dynamic_cast<SgOrthographicCamera*>(currentCamera)){
        GLfloat left, right, bottom, top;
        getViewVolume(*camera, left, right, bottom, top);
        glOrtho(left, right, bottom, top, camera->nearDistance(), camera->farDistance());
        
    } else {
        gluPerspective(40.0f, aspectRatio, 0.01, 1.0e4);
    }

    GLdouble P[16];
    glGetDoublev(GL_PROJECTION_MATRIX, P);
    lastProjectionMatrix = Eigen::Map<Eigen::Matrix<GLdouble, 4, 4> >(P);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // Render headlight
    if(!isPicking && defaultLighting){
        glEnable(GL_LIGHTING);

        if(!headLight->on()){
            glDisable(GL_LIGHT0);
            glDisable(GL_LIGHT1);
        } else {
            static const Affine3 I = Affine3::Identity();
            renderLight(headLight.get(), GL_LIGHT0, I);

            if(isHeadLightLightingFromBackEnabled){
                if(SgDirectionalLight* directionalHeadLight = dynamic_cast<SgDirectionalLight*>(headLight.get())){
                    SgDirectionalLight lightFromBack(*directionalHeadLight);
                    lightFromBack.setDirection(-directionalHeadLight->direction());
                    renderLight(&lightFromBack, GL_LIGHT1, I);
                }
            }
        }
    } else {
        glDisable(GL_LIGHTING);
    }

    Vstack.clear();
    Vstack.push_back((*cameras)[currentCameraIndex].M.inverse(Eigen::Isometry));
    const Affine3& V = Vstack.back();
    lastViewMatrix = V;
    glLoadMatrixd(V.data());
}


void GLSceneRendererImpl::getViewVolume
(const SgOrthographicCamera& camera, GLfloat& out_left, GLfloat& out_right, GLfloat& out_bottom, GLfloat& out_top) const
{
    GLfloat h = camera.height();
    out_top = h / 2.0f;
    out_bottom = -h / 2.0f;
    GLfloat w = h * aspectRatio;
    out_left = -w / 2.0f;
    out_right = w / 2.0f;
}


void GLSceneRendererImpl::renderLights()
{
    GLint maxLights;
    glGetIntegerv(GL_MAX_LIGHTS, &maxLights);
    maxLights -= numSystemLights;
    
    if(lights.size() > maxLights){
        lights.resize(maxLights);
    }
        
    for(size_t i=0; i < lights.size(); ++i){
        const LightInfo& info = lights[i];
        const GLint id = GL_LIGHT0 + numSystemLights + i;
        renderLight(info.light, id, info.M);
    }

    for(size_t i = lights.size(); i < prevNumLights; ++i){
        const GLint lightID = GL_LIGHT0 + numSystemLights + i;
        glDisable(lightID);
    }

    prevNumLights = lights.size();
}


void GLSceneRendererImpl::renderLight(const SgLight* light, GLint id, const Affine3& T)
{
    bool isValid = false;

    if(light->on()){
        
        if(const SgDirectionalLight* dirLight = dynamic_cast<const SgDirectionalLight*>(light)){

            isValid = true;
            
            Vector4f pos;
            pos << (T.linear() * -dirLight->direction()).cast<float>(), 0.0f;
            glLightfv(id, GL_POSITION, pos.data());
            
            /*
              glLightf(id, GL_CONSTANT_ATTENUATION, 0.0f);
              glLightf(id, GL_LINEAR_ATTENUATION, 0.0f);
              glLightf(id, GL_QUADRATIC_ATTENUATION, 0.0f);
            */
            
        } else if(const SgPointLight* pointLight = dynamic_cast<const SgPointLight*>(light)){
            
            isValid = true;
            
            Vector4f pos;
            pos << T.translation().cast<float>(), 1.0f;
            glLightfv(id, GL_POSITION, pos.data());
            
            glLightf(id, GL_CONSTANT_ATTENUATION, pointLight->constantAttenuation());
            glLightf(id, GL_LINEAR_ATTENUATION, pointLight->linearAttenuation());
            glLightf(id, GL_QUADRATIC_ATTENUATION, pointLight->quadraticAttenuation());
            
            if(const SgSpotLight* spotLight = dynamic_cast<const SgSpotLight*>(pointLight)){
                Vector3f direction = (T.linear() * spotLight->direction()).cast<GLfloat>();
                glLightfv(id, GL_SPOT_DIRECTION, direction.data());
                glLightf(id, GL_SPOT_CUTOFF, degree(spotLight->cutOffAngle()));
                glLightf(id, GL_SPOT_EXPONENT, 0.5f);
                
            } else {
                glLightf(id, GL_SPOT_CUTOFF, 180.0f);
            }
        }
    }
        
    if(!isValid){
        glDisable(id);
        
    } else {
        Vector4f diffuse;
        diffuse << (light->intensity() * light->color()), 1.0f;
        glLightfv(id, GL_DIFFUSE, diffuse.data());
        glLightfv(id, GL_SPECULAR, diffuse.data());
        
        Vector4f ambient;
        ambient << (light->ambientIntensity() * light->color()), 1.0f;
        glLightfv(id, GL_AMBIENT, ambient.data());
        
        glEnable(id);
    }
}


void GLSceneRenderer::endRendering()
{
    impl->endRendering();
}


void GLSceneRendererImpl::endRendering()
{
    if(isCheckingUnusedCaches){
        currentCacheMap->clear();
        hasValidNextCacheMap = true;
    }

    if(isNewDisplayListDoubleRenderingEnabled && isNewDisplayListCreated){
        scene->notifyUpdate();
    }
}


void GLSceneRenderer::render()
{
    impl->render();
}


void GLSceneRendererImpl::render()
{
    beginRendering(true);

    sceneRoot->accept(*self);

    if(!transparentShapeInfos.empty()){
        renderTransparentShapes();
    }

    endRendering();
}


void GLSceneRenderer::flush()
{
    glFlush();
}


bool GLSceneRenderer::pick(int x, int y)
{
    return impl->pick(x, y);
}


/*
  http://stackoverflow.com/questions/4040616/opengl-gl-select-or-manual-collision-detection
  http://content.gpwiki.org/index.php/OpenGL_Selection_Using_Unique_Color_IDs
  http://www.opengl-tutorial.org/miscellaneous/clicking-on-objects/picking-with-an-opengl-hack/
  http://www.codeproject.com/Articles/35139/Interactive-Techniques-in-Three-dimensional-Scenes#_OpenGL_Picking_by
  http://en.wikibooks.org/wiki/OpenGL_Programming/Object_selection

  Use a Framebuffer object?
*/
bool GLSceneRendererImpl::pick(int x, int y)
{
    glPushAttrib(GL_ENABLE_BIT);

    //glDisable(GL_LIGHTING); // disable this later in 'renderCamera()'
    glDisable(GL_BLEND);
    glDisable(GL_MULTISAMPLE);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DITHER);
    glDisable(GL_FOG);

    if(!SHOW_IMAGE_FOR_PICKING){
        glScissor(x, y, 1, 1);
        glEnable(GL_SCISSOR_TEST);
    }
    
    isPicking = true;
    render();
    isPicking = false;

    glPopAttrib();

    GLfloat color[4];
    glReadPixels(x, y, 1, 1, GL_RGBA, GL_FLOAT, color);
    if(SHOW_IMAGE_FOR_PICKING){
        color[2] = 0.0f;
    }
    unsigned int id = (color[0] * 255) + ((int)(color[1] * 255) << 8) + ((int)(color[2] * 255) << 16) - 1;

    pickedNodePath.clear();

    if(0 < id && id < pickingNodePathList.size()){
        pickedNodePath = *pickingNodePathList[id];
        GLfloat depth;
        glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
        GLdouble ox, oy, oz;
        gluUnProject(x, y, depth,
                     lastViewMatrix.matrix().data(),
                     lastProjectionMatrix.data(),
                     viewport.data(),
                     &ox, &oy, &oz);
        pickedPoint << ox, oy, oz;
    }

    return !pickedNodePath.empty();
}


const std::vector<SgNode*>& GLSceneRenderer::pickedNodePath() const
{
    return impl->pickedNodePath;
}


const Vector3& GLSceneRenderer::pickedPoint() const
{
    return impl->pickedPoint;
}



inline void GLSceneRendererImpl::setPickColor(unsigned int id)
{
    float r = (id & 0xff) / 255.0;
    float g = ((id >> 8) & 0xff) / 255.0;
    float b = ((id >> 16) & 0xff) / 255.0;
    if(SHOW_IMAGE_FOR_PICKING){
        b = 1.0f;
    }
    glColor4f(r, g, b, 1.0f);
    currentColor << r, g, b, 1.0f;
}
        

/**
   @return id of the current object
*/
inline unsigned int GLSceneRendererImpl::pushPickName(SgNode* node, bool doSetColor)
{
    unsigned int id = 0;
    
    if(isPicking && !isCompiling){
        id = pickingNodePathList.size() + 1;
        currentNodePath.push_back(node);
        pickingNodePathList.push_back(boost::make_shared<SgNodePath>(currentNodePath));
        if(doSetColor){
            setPickColor(id);
        }
    }

    return id;
}


inline void GLSceneRendererImpl::popPickName()
{
    if(isPicking && !isCompiling){
        currentNodePath.pop_back();
    }
}


void GLSceneRenderer::visitGroup(SgGroup* group)
{
    impl->pushPickName(group);
    SceneVisitor::visitGroup(group);
    impl->popPickName();
}


void GLSceneRenderer::visitInvariantGroup(SgInvariantGroup* group)
{
    impl->visitInvariantGroup(group);
}


void GLSceneRendererImpl::visitInvariantGroup(SgInvariantGroup* group)
{
    if(!USE_DISPLAY_LISTS || isCompiling){
        self->visitGroup(group);

    } else {
        ShapeCache* cache;
        CacheMap::iterator p = currentCacheMap->find(group);
        if(p == currentCacheMap->end()){
            cache = new ShapeCache();
            currentCacheMap->insert(CacheMap::value_type(group, cache));
        } else {
            cache = static_cast<ShapeCache*>(p->second.get());
        }

        if(!cache->listID && !isPicking){
            currentShapeCache = cache;
            currentShapeCacheTopViewMatrixIndex = Vstack.size() - 1;

            cache->listID = glGenLists(1);
            if(cache->listID){
                glNewList(cache->listID, GL_COMPILE);

                isCompiling = true;
                clearGLState();
                self->visitGroup(group);
                isCompiling = false;

                if(stateFlag[LIGHTING] || stateFlag[CURRENT_COLOR]){
                    cache->useIDforPicking = true;
                }
                glEndList();

                isNewDisplayListCreated = true;
            }
        }

        GLuint listID = cache->listID;

        if(listID){
            if(isPicking && cache->useIDforPicking){
                if(!cache->listIDforPicking){
                    currentShapeCache = cache;
                    currentShapeCacheTopViewMatrixIndex = Vstack.size() - 1;
                    cache->listIDforPicking = glGenLists(1);
                    if(cache->listIDforPicking){
                        glNewList(cache->listIDforPicking, GL_COMPILE);
                        isCompiling = true;
                        clearGLState();
                        self->visitGroup(group);
                        isCompiling = false;
                        glEndList();
                    }
                }
                listID = cache->listIDforPicking;
            }
            if(listID){
                const unsigned int pickId = pushPickName(group);
                glPushAttrib(GL_ENABLE_BIT);
                glCallList(listID);
                glPopAttrib();
                clearGLState();
                popPickName();

                const vector<TransparentShapeInfoPtr>& transparentShapes = cache->transparentShapes;
                if(!transparentShapes.empty()){
                    const Affine3& V = Vstack.back();
                    for(size_t i=0; i < transparentShapes.size(); ++i){
                        const TransparentShapeInfo& src = *transparentShapes[i];
                        TransparentShapeInfoPtr info = make_shared_aligned<TransparentShapeInfo>();
                        info->shape = src.shape;
                        info->pickId = pickId;
                        info->V = V * src.V;
                        transparentShapeInfos.push_back(info);
                    }
                }
            }

            if(isCheckingUnusedCaches){
                nextCacheMap->insert(CacheMap::value_type(group, cache));
            }
        }
    }
    currentShapeCache = 0;
}


void GLSceneRenderer::visitTransform(SgTransform* transform)
{
    Affine3 T;
    transform->getTransform(T);

    Affine3Array& Vstack = impl->Vstack;
    Vstack.push_back(Vstack.back() * T);

    glPushMatrix();
    glMultMatrixd(T.data());

    /*
      if(isNotRotationMatrix){
      glPushAttrib(GL_ENABLE_BIT);
      glEnable(GL_NORMALIZE);
      }
    */
    
    visitGroup(transform);
    
    /*
      if(isNotRotationMatrix){
      glPopAttrib();
      }
    */
    
    glPopMatrix();
    Vstack.pop_back();
}


void GLSceneRendererImpl::visitShape(SgShape* shape)
{
    SgMesh* mesh = shape->mesh();
    if(mesh){
        if(mesh->hasVertices()){
            SgMaterial* material = shape->material();
            SgTexture* texture = isTextureEnabled ? shape->texture() : 0;

            if((material && material->transparency() > 0.0)
               /* || (texture && texture->constImage().hasAlphaComponent()) */){
                // A transparent shape is rendered later
                TransparentShapeInfoPtr info = make_shared_aligned<TransparentShapeInfo>();
                info->shape = shape;
                if(isCompiling){
                    info->V = Vstack[currentShapeCacheTopViewMatrixIndex].inverse() * Vstack.back();
                    currentShapeCache->transparentShapes.push_back(info);
                } else {
                    info->V = Vstack.back();
                    info->pickId = pushPickName(shape, false);
                    popPickName();
                    transparentShapeInfos.push_back(info);
                }
            } else {
                pushPickName(shape);
                bool hasTexture = false;
                if(!isPicking){
                    renderMaterial(material);
                    if(texture && mesh->hasTexCoords()){
                        hasTexture = renderTexture(texture, material);
                    }
                }
                renderMesh(mesh, hasTexture);
                popPickName();
            }
        }
    }
}


void GLSceneRenderer::visitUnpickableGroup(SgUnpickableGroup* group)
{
    if(!impl->isPicking){
        visitGroup(group);
    }
}


void GLSceneRenderer::visitShape(SgShape* shape)
{
    impl->visitShape(shape);
}


void GLSceneRendererImpl::renderMaterial(const SgMaterial* material)
{
    if(!material){
        material = defaultMaterial;
    }
    
    float alpha = 1.0 - material->transparency();

    Vector4f color;
    color << material->diffuseColor(), alpha;
    setDiffuseColor(color);
        
    color.head<3>() *= material->ambientIntensity();
    setAmbientColor(color);

    color << material->emissiveColor(), alpha;
    setEmissionColor(color);
    
    color << material->specularColor(), alpha;
    setSpecularColor(color);
    
    float shininess = (127.0 * material->shininess()) + 1.0;
    setShininess(shininess);

    lastAlpha = alpha;
}


bool GLSceneRendererImpl::renderTexture(SgTexture* texture, bool withMaterial)
{
    SgImage* sgImage = texture->image();
    if(!sgImage || sgImage->empty()){
        return false;
    }

    const Image& image = sgImage->constImage();
    const int width = image.width();
    const int height = image.height();
    bool doLoadTexImage = false;
    bool doReloadTexImage = false;

    CacheMap::iterator p = currentCacheMap->find(sgImage);
    TextureCache* cache;
    if(p != currentCacheMap->end()){
        cache = static_cast<TextureCache*>(p->second.get());
    } else {
        cache = new TextureCache;
        currentCacheMap->insert(CacheMap::value_type(sgImage, cache));
    }
    if(cache->isBound){
        glBindTexture(GL_TEXTURE_2D, cache->textureName);
        if(cache->isImageUpdateNeeded){
            doLoadTexImage = true;
            doReloadTexImage = cache->isSameSizeAs(image);
        }
    } else {
        glGenTextures(1, &cache->textureName);
        glBindTexture(GL_TEXTURE_2D, cache->textureName);
        cache->isBound = true;
        doLoadTexImage = true;
    }
    if(isCheckingUnusedCaches){
        nextCacheMap->insert(CacheMap::value_type(sgImage, cache));
    }
    cache->width = width;
    cache->height = height;
    cache->numComponents = image.numComponents();
    cache->isImageUpdateNeeded = false;
    
    const bool useMipmap = isCompiling;

    if(doLoadTexImage){
        GLint internalFormat = GL_RGB;
        GLenum format = GL_RGB;
        
        switch(image.numComponents()){
        case 1 : internalFormat = GL_LUMINANCE;
            format = GL_LUMINANCE;
            break;
        case 2 : internalFormat = GL_LUMINANCE_ALPHA;
            format = GL_LUMINANCE_ALPHA;
            break;
        case 3 : internalFormat = GL_RGB;
            format = GL_RGB;
            break;
        case 4 : internalFormat = GL_RGBA;
            format = GL_RGBA;
            break;
        default :
            return false;
        }
        
        if(image.numComponents() == 3){
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        } else {
            glPixelStorei(GL_UNPACK_ALIGNMENT, image.numComponents());
        }

        if(useMipmap){
            gluBuild2DMipmaps(
                GL_TEXTURE_2D, internalFormat, width, height, format, GL_UNSIGNED_BYTE, image.pixels());
        } else {
            if(doReloadTexImage){
                glTexSubImage2D(
                    GL_TEXTURE_2D, 0, 0, 0, width, height, format, GL_UNSIGNED_BYTE, image.pixels());
            } else {
                // gluScaleImage(...);
                glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, width, height, 0,
                             format, GL_UNSIGNED_BYTE, image.pixels());
            }
        }
    }

    if(useMipmap){
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    } else {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    }
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, texture->repeatS() ? GL_REPEAT : GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, texture->repeatT() ? GL_REPEAT : GL_CLAMP);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, withMaterial ? GL_MODULATE : GL_REPLACE);
    
    if(SgTextureTransform* tt = texture->textureTransform()){
        glMatrixMode(GL_TEXTURE);
        glLoadIdentity();
        glTranslated(-tt->center()[0], -tt->center()[1], 0.0 );
        glScaled(tt->scale()[0], tt->scale()[1], 0.0 );
        glRotated(tt->rotation(), 0.0, 0.0, 1.0 );
        glTranslated(tt->center()[0], tt->center()[1], 0.0 );
        glTranslated(tt->translation()[0], tt->translation()[1], 0.0 );
        glMatrixMode(GL_MODELVIEW);
    } else {
        glMatrixMode(GL_TEXTURE);
        glLoadIdentity();
        glMatrixMode(GL_MODELVIEW);
    }

    return true;
}


/**
   \todo sort the shape nodes by the distance from the viewpoint
*/
void GLSceneRendererImpl::renderTransparentShapes()
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();;
    
    if(!isPicking){
        enableBlend(true);
    }
    
    const int n = transparentShapeInfos.size();
    for(int i=0; i < n; ++i){
        TransparentShapeInfo& info = *transparentShapeInfos[i];
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixd(info.V.data());
        SgShape* shape = info.shape;
        bool hasTexture = false;
        if(isPicking){
            setPickColor(info.pickId);
        } else {
            renderMaterial(shape->material());
            SgTexture* texture = isTextureEnabled ? shape->texture() : 0;
            if(texture && shape->mesh()->hasTexCoords()){
                hasTexture = renderTexture(texture, shape->material());
            }
        }
        renderMesh(shape->mesh(), hasTexture);
    }

    if(!isPicking){
        enableBlend(false);
    }

    transparentShapeInfos.clear();

    glPopMatrix();
}


void GLSceneRendererImpl::putMeshData(SgMesh* mesh)
{
    if(!mesh->hasColors()){
        return;
    }
        
    if(mesh->hasVertices()){
        cout << "vertices: \n";
        SgVertexArray& vertices = *mesh->vertices();
        for(size_t i=0; i < vertices.size(); ++i){
            const Vector3f& v = vertices[i];
            cout << "(" << v.x() << ", " << v.y() << ", " << v.z() << "), ";
        }
        cout << "\n";
    }
    if(!mesh->triangleVertices().empty()){
        cout << "triangles: \n";
        const int n = mesh->numTriangles();
        for(int i=0; i < n; ++i){
            SgMesh::TriangleRef t = mesh->triangle(i);
            cout << "(" << t[0] << ", " << t[1] << ", " << t[2] << "), ";
        }
        cout << "\n";
    }
    if(mesh->hasNormals()){
        cout << "normals: \n";
        SgNormalArray& normals = *mesh->normals();
        for(size_t i=0; i < normals.size(); ++i){
            const Vector3f& v = normals[i];
            cout << "(" << v.x() << ", " << v.y() << ", " << v.z() << "), ";
        }
        cout << "\n";
        SgIndexArray& indices = mesh->normalIndices();
        if(!indices.empty()){
            cout << "normalIndices: \n";
            for(size_t i=0; i < indices.size(); ++i){
                cout << indices[i] << ", ";
            }
            cout << "\n";
        }
    }
    if(mesh->hasColors()){
        cout << "colors: \n";
        SgColorArray& colors = *mesh->colors();
        for(size_t i=0; i < colors.size(); ++i){
            const Vector3f& c = colors[i];
            cout << "(" << c.x() << ", " << c.y() << ", " << c.z() << "), ";
        }
        cout << "\n";
        SgIndexArray& indices = mesh->colorIndices();
        if(!indices.empty()){
            cout << "colorIndices: \n";
            for(size_t i=0; i < indices.size(); ++i){
                cout << indices[i] << ", ";
            }
            cout << "\n";
        }
    }
    cout << endl;
}


void GLSceneRendererImpl::renderMesh(SgMesh* mesh, bool hasTexture)
{
    if(false){
        putMeshData(mesh);
    }
    
    const bool ENABLE_CULLING = true;
    if(ENABLE_CULLING){
        enableCullFace(mesh->isSolid());
        setLightModelTwoSide(!mesh->isSolid());
    } else {
        enableCullFace(false);
        setLightModelTwoSide(true);
    }

    glPushClientAttrib(GL_CLIENT_VERTEX_ARRAY_BIT);

    if(!USE_VBO){
        writeVertexBuffers(mesh, 0, hasTexture);

    } else {
        ShapeCache* cache;
        CacheMap::iterator it = currentCacheMap->find(mesh);
        if(it != currentCacheMap->end()){
            cache = static_cast<ShapeCache*>(it->second.get());
        } else {
            it = currentCacheMap->insert(CacheMap::value_type(mesh, new ShapeCache)).first;
            cache = static_cast<ShapeCache*>(it->second.get());
            writeVertexBuffers(mesh, cache, hasTexture);
        }
        if(isCheckingUnusedCaches){
            nextCacheMap->insert(*it);
        }
        if(cache->vertexBufferName() != GL_INVALID_VALUE){
            glEnableClientState(GL_VERTEX_ARRAY);
            glBindBuffer(GL_ARRAY_BUFFER, cache->vertexBufferName());
            glVertexPointer(3, GL_FLOAT, 0, 0);
                        
            if(cache->normalBufferName() != GL_INVALID_VALUE){
                glEnableClientState(GL_NORMAL_ARRAY);
                glBindBuffer(GL_ARRAY_BUFFER, cache->normalBufferName());
                glNormalPointer(GL_FLOAT, 0, 0);
            }
            if(cache->texCoordBufferName() != GL_INVALID_VALUE){
                glEnableClientState(GL_TEXTURE_COORD_ARRAY);
                glBindBuffer(GL_ARRAY_BUFFER, cache->texCoordBufferName());
                glTexCoordPointer(2, GL_FLOAT, 0, 0);
                glEnable(GL_TEXTURE_2D);
            }
            if(USE_INDEXING){
                if(cache->indexBufferName() != GL_INVALID_VALUE){
                    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cache->indexBufferName());
                    glDrawElements(GL_TRIANGLES, cache->size, GL_UNSIGNED_INT, 0);
                }
            } else {
                glDrawArrays(GL_TRIANGLES, 0, cache->size);
            }
            if(cache->texCoordBufferName() != GL_INVALID_VALUE){
                glDisable(GL_TEXTURE_2D);
            }
        }
    }

    glPopClientAttrib();
}


void GLSceneRendererImpl::writeVertexBuffers(SgMesh* mesh, ShapeCache* cache, bool hasTexture)
{
    SgVertexArray& orgVertices = *mesh->vertices();
    SgIndexArray& orgTriangleVertices = mesh->triangleVertices();
    const size_t numTriangles = mesh->numTriangles();
    const size_t totalNumVertices = orgTriangleVertices.size();
    const bool hasNormals = mesh->hasNormals() && !isPicking;
    const bool hasColors = mesh->hasColors() && !isPicking;
    SgVertexArray* vertices = 0;
    SgNormalArray* normals = 0;
    SgIndexArray* triangleVertices = 0;
    ColorArray* colors = 0;
    SgTexCoordArray* texCoords = 0;

    if(USE_INDEXING){
        vertices = &orgVertices;
        triangleVertices = &orgTriangleVertices;
        if(hasNormals){
            if(mesh->normalIndices().empty() && mesh->normals()->size() == orgVertices.size()){
                normals = mesh->normals();
            } else {
                normals = &buf->normals;
                normals->resize(orgVertices.size());
            }
        }
        if(hasTexture){
            if(mesh->texCoordIndices().empty() && mesh->texCoords()->size() == orgVertices.size()){
                texCoords = mesh->texCoords();
            } else {
                texCoords = &buf->texCoords;
                texCoords->resize(orgVertices.size());
            }
        }
    } else {
        vertices = &buf->vertices;
        vertices->clear();
        vertices->reserve(totalNumVertices);
        if(hasNormals){
            normals = &buf->normals;
            normals->clear();
            normals->reserve(totalNumVertices);
        }
        if(hasColors){
            colors = &buf->colors;
            colors->clear();
            colors->reserve(totalNumVertices);
        }
        if(hasTexture){
            texCoords = &buf->texCoords;
            texCoords->clear();
            texCoords->reserve(totalNumVertices);
        }
    }
    
    int faceVertexIndex = 0;
    int numFaceVertices = 0;
    
    for(size_t i=0; i < numTriangles; ++i){
        for(size_t j=0; j < 3; ++j){
            const int orgVertexIndex = orgTriangleVertices[faceVertexIndex];
            if(!USE_INDEXING){
                vertices->push_back(orgVertices[orgVertexIndex]);
            }
            if(hasNormals){
                if(mesh->normalIndices().empty()){
                    if(!USE_INDEXING){
                        normals->push_back(mesh->normals()->at(orgVertexIndex));
                    }
                } else {
                    const int normalIndex = mesh->normalIndices()[faceVertexIndex];
                    if(USE_INDEXING){
                        normals->at(orgVertexIndex) = mesh->normals()->at(normalIndex);
                    } else {
                        normals->push_back(mesh->normals()->at(normalIndex));
                    }
                }
            }
            if(hasColors){
                if(mesh->colorIndices().empty()){
                    if(!USE_INDEXING){
                        colors->push_back(createColorWithAlpha(mesh->colors()->at(faceVertexIndex)));
                    }
                } else {
                    const int colorIndex = mesh->colorIndices()[faceVertexIndex];
                    if(USE_INDEXING){
                        colors->at(orgVertexIndex) = createColorWithAlpha(mesh->colors()->at(colorIndex));
                    } else {
                        colors->push_back(createColorWithAlpha(mesh->colors()->at(colorIndex)));
                    }
                }
            }
            if(hasTexture){
                if(mesh->texCoordIndices().empty()){
                    if(!USE_INDEXING){
                        texCoords->push_back(mesh->texCoords()->at(orgVertexIndex));
                    }
                }else{
                    const int texCoordIndex = mesh->texCoordIndices()[faceVertexIndex];
                    if(USE_INDEXING){
                        texCoords->at(orgVertexIndex) = mesh->texCoords()->at(texCoordIndex);
                    } else {
                        texCoords->push_back(mesh->texCoords()->at(texCoordIndex));
                    }
                }
            }
            ++faceVertexIndex;
        }
    }

    if(USE_VBO){
        if(cache->vertexBufferName() == GL_INVALID_VALUE){
            glGenBuffers(1, &cache->vertexBufferName());
            glBindBuffer(GL_ARRAY_BUFFER, cache->vertexBufferName());
            glBufferData(GL_ARRAY_BUFFER, vertices->size() * sizeof(Vector3f), vertices->data(), GL_STATIC_DRAW);
        }
    } else {
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, vertices->data());
    }
    if(normals){
        if(USE_VBO){
            if(cache->normalBufferName() == GL_INVALID_VALUE){
                glGenBuffers(1, &cache->normalBufferName());
                glBindBuffer(GL_ARRAY_BUFFER, cache->normalBufferName());
                glBufferData(GL_ARRAY_BUFFER, normals->size() * sizeof(Vector3f), normals->data(), GL_STATIC_DRAW);
            }
        } else {
            glEnableClientState(GL_NORMAL_ARRAY);
            glNormalPointer(GL_FLOAT, 0, normals->data());
        }
    }
    bool useColorArray = false;
    if(colors){
        if(!USE_VBO){
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
            glEnable(GL_COLOR_MATERIAL);
            glEnableClientState(GL_COLOR_ARRAY);
            glColorPointer(4, GL_FLOAT, 0, &colors[0][0]);
            useColorArray = true;
        }
    }
    if(hasTexture){
        if(USE_VBO){
            if(cache->texCoordBufferName() == GL_INVALID_VALUE){
                glGenBuffers(1, &cache->texCoordBufferName());
                glBindBuffer(GL_ARRAY_BUFFER, cache->texCoordBufferName());
                glBufferData(GL_ARRAY_BUFFER, texCoords->size() * sizeof(Vector2f), texCoords->data(), GL_STATIC_DRAW);
            }
        } else {
            glEnableClientState(GL_TEXTURE_COORD_ARRAY);
            glTexCoordPointer(2, GL_FLOAT, 0, texCoords->data());
        }
        glEnable(GL_TEXTURE_2D);
    }

    if(USE_VBO){
        if(!triangleVertices){
            cache->size = vertices->size();

        } else if(cache->indexBufferName() == GL_INVALID_VALUE){
            glGenBuffers(1, &cache->indexBufferName());
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cache->indexBufferName());
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, triangleVertices->size(), &triangleVertices->front(), GL_STATIC_DRAW);
            cache->size = triangleVertices->size();
        }
            
    } else {
        if(USE_INDEXING){
            glDrawElements(GL_TRIANGLES, triangleVertices->size(), GL_UNSIGNED_INT, &triangleVertices->front());
        } else {
            glDrawArrays(GL_TRIANGLES, 0, vertices->size());
        }
    }

    if(useColorArray){
        glDisable(GL_COLOR_MATERIAL);
        stateFlag.set(CURRENT_COLOR);
    }
    if(hasTexture){
        glDisable(GL_TEXTURE_2D);
    }

    if(doNormalVisualization && !isPicking){
        enableLighting(false);
        if(!USE_INDEXING){
            vector<Vector3f> lines;
            for(size_t i=0; i < vertices->size(); ++i){
                const Vector3f& v = (*vertices)[i];
                lines.push_back(v);
                lines.push_back(v + (*normals)[i] * normalLength);
            }
            glDisableClientState(GL_NORMAL_ARRAY);
            glVertexPointer(3, GL_FLOAT, 0, lines.front().data());
            setColor(Vector4f(0.0f, 1.0f, 0.0f, 1.0f));
            glDrawArrays(GL_LINES, 0, lines.size());
        }
        enableLighting(true);
    }
}


void GLSceneRenderer::visitPointSet(SgPointSet* pointSet)
{
    impl->visitPointSet(pointSet);
}


void GLSceneRendererImpl::visitPointSet(SgPointSet* pointSet)
{
    if(!pointSet->hasVertices()){
        return;
    }
    const double s = pointSet->pointSize();
    if(s > 0.0){
        setPointSize(s);
    }
    renderPlot(pointSet, *pointSet->vertices(), (GLenum)GL_POINTS);
    if(s > 0.0){
        setPointSize(s);
    }
}


void GLSceneRendererImpl::renderPlot(SgPlot* plot, SgVertexArray& expandedVertices, GLenum primitiveMode)
{
    glPushClientAttrib(GL_CLIENT_VERTEX_ARRAY_BIT);
        
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, expandedVertices.data());
    
    SgMaterial* material = plot->material() ? plot->material() : defaultMaterial.get();
    
    if(!plot->hasNormals()){
        enableLighting(false);
        //glDisableClientState(GL_NORMAL_ARRAY);
        lastAlpha = 1.0;
        if(!plot->hasColors()){
            setColor(createColorWithAlpha(material->diffuseColor()));
        }
    } else if(!isPicking){
        enableCullFace(false);
        setLightModelTwoSide(true);
        renderMaterial(material);
        
        const SgNormalArray& orgNormals = *plot->normals();
        SgNormalArray& normals = buf->normals;
        const SgIndexArray& normalIndices = plot->normalIndices();
        if(normalIndices.empty()){
            normals = orgNormals;
        } else {
            normals.clear();
            normals.reserve(normalIndices.size());
            for(int i=0; i < normalIndices.size(); ++i){
                normals.push_back(orgNormals[normalIndices[i]]);
            }
        }
        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, 0, normals.data());
    }

    bool isColorMaterialEnabled = false;

    if(plot->hasColors() && !isPicking){
        const SgColorArray& orgColors = *plot->colors();
        ColorArray& colors = buf->colors;
        colors.clear();
        colors.reserve(expandedVertices.size());
        const SgIndexArray& colorIndices = plot->colorIndices();
        if(colorIndices.empty()){
            for(int i=0; i < orgColors.size(); ++i){
                colors.push_back(createColorWithAlpha(orgColors[i]));
            }
        } else {
            for(int i=0; i < colorIndices.size(); ++i){
                colors.push_back(createColorWithAlpha(orgColors[colorIndices[i]]));
            }
        }
        if(plot->hasNormals()){
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
            glEnable(GL_COLOR_MATERIAL);
            isColorMaterialEnabled = true;
        }
        glEnableClientState(GL_COLOR_ARRAY);
        glColorPointer(4, GL_FLOAT, 0, &colors[0][0]);
        stateFlag.set(CURRENT_COLOR);
        //setColor(colors.back()); // set the last color
    }
    
    pushPickName(plot);
    glDrawArrays(primitiveMode, 0, expandedVertices.size());
    popPickName();
    
    if(plot->hasNormals()){
        if(isColorMaterialEnabled){
            glDisable(GL_COLOR_MATERIAL);
        }
    } else {
        enableLighting(true);
    }
    
    glPopClientAttrib();
}


void GLSceneRenderer::visitLineSet(SgLineSet* lineSet)
{
    impl->visitLineSet(lineSet);
}


void GLSceneRendererImpl::visitLineSet(SgLineSet* lineSet)
{
    const int n = lineSet->numLines();
    if(!lineSet->hasVertices() || (n <= 0)){
        return;
    }

    const SgVertexArray& orgVertices = *lineSet->vertices();
    SgVertexArray& vertices = buf->vertices;
    vertices.clear();
    vertices.reserve(n * 2);
    for(int i=0; i < n; ++i){
        SgLineSet::LineRef line = lineSet->line(i);
        vertices.push_back(orgVertices[line[0]]);
        vertices.push_back(orgVertices[line[1]]);
    }

    const double w = lineSet->lineWidth();
    if(w > 0.0){
        setLineWidth(w);
    }
    renderPlot(lineSet, vertices, GL_LINES);
    if(w > 0.0){
        setLineWidth(defaultLineWidth);
    }
}


void GLSceneRenderer::visitPreprocessed(SgPreprocessed* preprocessed)
{

}


void GLSceneRenderer::visitLight(SgLight* light)
{

}


void GLSceneRenderer::visitOverlay(SgOverlay* overlay)
{
    if(isPicking()){
        return;
    }
    
    glPushAttrib(GL_LIGHTING_BIT);
    glDisable(GL_LIGHTING);
            
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    SgOverlay::ViewVolume v;
    v.left = -1.0;
    v.right = 1.0;
    v.bottom = -1.0;
    v.top = 1.0;
    v.zNear = 1.0;
    v.zFar = -1.0;

    overlay->calcViewVolume(impl->viewport[2], impl->viewport[3], v);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(v.left, v.right, v.bottom, v.top, v.zNear, v.zFar);

    visitGroup(overlay);
    
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    
    glPopAttrib();
}


bool GLSceneRenderer::isPicking()
{
    return impl->isPicking;
}


void GLSceneRendererImpl::clearGLState()
{
    stateFlag.reset();
    
    //! \todo get the current state from a GL context
    diffuseColor << 0.0f, 0.0f, 0.0f, 0.0f;
    ambientColor << 0.0f, 0.0f, 0.0f, 0.0f;
    emissionColor << 0.0f, 0.0f, 0.0f, 0.0f;
    specularColor << 0.0f, 0.0f, 0.0f, 0.0f;
    shininess = 0.0f;
    lastAlpha = 1.0f;
    isColorMaterialEnabled = false;
    isCullFaceEnabled = true;
    isCCW = false;
    isLightingEnabled = true;
    isLightModelTwoSide = false;
    isBlendEnabled = true;
    isDepthMaskEnabled = true;

    pointSize = defaultPointSize;
    lineWidth = defaultLineWidth;
}


void GLSceneRendererImpl::setColor(const Vector4f& color)
{
    if(!isPicking){
        if(!stateFlag[CURRENT_COLOR] || color != currentColor){
            glColor4f(color[0], color[1], color[2], color[3]);
            currentColor = color;
            stateFlag.set(CURRENT_COLOR);
        }
    }
}


void GLSceneRenderer::setColor(const Vector4f& color)
{
    impl->setColor(color);
}


void GLSceneRendererImpl::enableColorMaterial(bool on)
{
    if(!isPicking){
        if(!stateFlag[COLOR_MATERIAL] || isColorMaterialEnabled != on){
            if(on){
                glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
                glEnable(GL_COLOR_MATERIAL);
            } else {
                glDisable(GL_COLOR_MATERIAL);
            }
            isColorMaterialEnabled = on;
            stateFlag.set(COLOR_MATERIAL);
        }
    }
}


void GLSceneRenderer::enableColorMaterial(bool on)
{
    impl->enableColorMaterial(on);
}


void GLSceneRendererImpl::setDiffuseColor(const Vector4f& color)
{
    if(!stateFlag[DIFFUSE_COLOR] || diffuseColor != color){
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color.data());
        diffuseColor = color;
        stateFlag.set(DIFFUSE_COLOR);
    }
}


void GLSceneRenderer::setDiffuseColor(const Vector4f& color)
{
    impl->setDiffuseColor(color);
}


void GLSceneRendererImpl::setAmbientColor(const Vector4f& color)
{
    if(!stateFlag[AMBIENT_COLOR] || ambientColor != color){
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color.data());
        ambientColor = color;
        stateFlag.set(AMBIENT_COLOR);
    }
}


void GLSceneRenderer::setAmbientColor(const Vector4f& color)
{
    impl->setAmbientColor(color);
}


void GLSceneRendererImpl::setEmissionColor(const Vector4f& color)
{
    if(!stateFlag[EMISSION_COLOR] || emissionColor != color){
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, color.data());
        emissionColor = color;
        stateFlag.set(EMISSION_COLOR);
    }
}


void GLSceneRenderer::setEmissionColor(const Vector4f& color)
{
    impl->setEmissionColor(color);
}


void GLSceneRendererImpl::setSpecularColor(const Vector4f& color)
{
    if(!stateFlag[SPECULAR_COLOR] || specularColor != color){
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, color.data());
        specularColor = color;
        stateFlag.set(SPECULAR_COLOR);
    }
}


void GLSceneRenderer::setSpecularColor(const Vector4f& color)
{
    impl->setSpecularColor(color);
}


void GLSceneRendererImpl::setShininess(float s)
{
    if(!stateFlag[SHININESS] || shininess != s){
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, s);
        shininess = s;
        stateFlag.set(SHININESS);
    }
}


void GLSceneRenderer::setShininess(float s)
{
    impl->setShininess(s);
}


void GLSceneRendererImpl::enableCullFace(bool on)
{
    if(!stateFlag[CULL_FACE] || isCullFaceEnabled != on){
        if(on){
            glEnable(GL_CULL_FACE);
        } else {
            glDisable(GL_CULL_FACE);
        }
        isCullFaceEnabled = on;
        stateFlag.set(CULL_FACE);
    }
}


void GLSceneRenderer::enableCullFace(bool on)
{
    impl->enableCullFace(on);
}


void GLSceneRendererImpl::setFrontCCW(bool on)
{
    if(!stateFlag[CCW] || isCCW != on){
        if(on){
            glFrontFace(GL_CCW);
        } else {
            glFrontFace(GL_CW);
        }
        isCCW = on;
        stateFlag.set(CCW);
    }
}


void GLSceneRenderer::setFrontCCW(bool on)
{
    impl->setFrontCCW(on);
}


/**
   Lighting should not be enabled in rendering code
   which may be rendered with displaylists.
*/
void GLSceneRendererImpl::enableLighting(bool on)
{
    if(isPicking || !defaultLighting){
        return;
    }
    if(!stateFlag[LIGHTING] || isLightingEnabled != on){
        if(on){
            glEnable(GL_LIGHTING);
        } else {
            glDisable(GL_LIGHTING);
        }
        isLightingEnabled = on;
        stateFlag.set(LIGHTING);
    }
}


void GLSceneRenderer::enableLighting(bool on)
{
    impl->enableLighting(on);
}


void GLSceneRendererImpl::setLightModelTwoSide(bool on)
{
    if(!stateFlag[LIGHT_MODEL_TWO_SIDE] || isLightModelTwoSide != on){
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, on ? GL_TRUE : GL_FALSE);
        isLightModelTwoSide = on;
        stateFlag.set(LIGHT_MODEL_TWO_SIDE);
    }
}


void GLSceneRenderer::setLightModelTwoSide(bool on)
{
    impl->setLightModelTwoSide(on);
}


void GLSceneRendererImpl::enableBlend(bool on)
{
    if(isPicking){
        return;
    }
    if(!stateFlag[BLEND] || isBlendEnabled != on){
        if(on){
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            enableDepthMask(false);
        } else {
            glDisable(GL_BLEND);
            enableDepthMask(true);
        }
        isBlendEnabled = on;
        stateFlag.set(BLEND);
    }
}


void GLSceneRenderer::enableBlend(bool on)
{
    impl->enableBlend(on);
}


void GLSceneRendererImpl::enableDepthMask(bool on)
{
    if(!stateFlag[DEPTH_MASK] || isDepthMaskEnabled != on){
        glDepthMask(on);
        isDepthMaskEnabled = on;
        stateFlag.set(DEPTH_MASK);
    }
}


void GLSceneRenderer::enableDepthMask(bool on)
{
    impl->enableDepthMask(on);
}


void GLSceneRendererImpl::setPointSize(float size)
{
    if(!stateFlag[POINT_SIZE] || pointSize != size){
        if(isPicking){
            glPointSize(std::max(size, MinLineWidthForPicking));
        } else {
            glPointSize(size);
        }
        pointSize = size;
        stateFlag.set(POINT_SIZE);
    }
}


void GLSceneRenderer::setPointSize(float size)
{
    impl->setPointSize(size);
}


void GLSceneRendererImpl::setLineWidth(float width)
{
    if(!stateFlag[LINE_WIDTH] || lineWidth != width){
        if(isPicking){
            glLineWidth(std::max(width, MinLineWidthForPicking));
        } else {
            glLineWidth(width);
        }
        lineWidth = width;
        stateFlag.set(LINE_WIDTH);
    }
}


void GLSceneRenderer::setLineWidth(float width)
{
    impl->setLineWidth(width);
}



SgObject* SgCustomGLNode::clone(SgCloneMap& cloneMap) const
{
    return new SgCustomGLNode(*this, cloneMap);
}


void SgCustomGLNode::accept(SceneVisitor& visitor)
{
    GLSceneRenderer* renderer = dynamic_cast<GLSceneRenderer*>(&visitor);
    if(renderer){
        renderer->impl->pushPickName(this);
        render(*renderer);
        renderer->impl->popPickName();
    } else {
        visitor.visitGroup(this);
    }
}
    

void SgCustomGLNode::render(GLSceneRenderer& renderer)
{
    if(renderingFunction){
        renderingFunction(renderer);
    }
}


void SgCustomGLNode::setRenderingFunction(RenderingFunction f)
{
    renderingFunction = f;
}



const Vector3f& GLSceneRenderer::backgroundColor() const
{
    return impl->bgColor;
}


void GLSceneRenderer::setBackgroundColor(const Vector3f& color)
{
    impl->bgColor = color;
}


SgLight* GLSceneRenderer::headLight()
{
    return impl->headLight.get();
}


void GLSceneRenderer::setHeadLight(SgLight* light)
{
    impl->headLight = light;
}


void GLSceneRenderer::setHeadLightLightingFromBackEnabled(bool on)
{
    impl->isHeadLightLightingFromBackEnabled = on;
}


void GLSceneRenderer::setAsDefaultLight(SgLight* light)
{
    impl->defaultLights.insert(light);
}


void GLSceneRenderer::unsetDefaultLight(SgLight* light)
{
    impl->defaultLights.erase(light);
}


void GLSceneRenderer::enableAdditionalLights(bool on)
{
    impl->additionalLightsEnabled = on;
}


void GLSceneRenderer::setPolygonMode(PolygonMode mode)
{
    impl->polygonMode = mode;
}


void GLSceneRenderer::setDefaultLighting(bool on)
{
    if(on != impl->defaultLighting){
        impl->defaultLighting = on;
        requestToClearCache();
    }
}


void GLSceneRenderer::setDefaultSmoothShading(bool on)
{
    impl->defaultSmoothShading = on;
}


SgMaterial* GLSceneRenderer::defaultMaterial()
{
    return impl->defaultMaterial.get();
}


void GLSceneRenderer::setDefaultColor(const Vector4f& color)
{
    impl->defaultColor = color;
}


void GLSceneRenderer::enableTexture(bool on)
{
    if(on != impl->isTextureEnabled){
        impl->isTextureEnabled = on;
        requestToClearCache();
    }
}


void GLSceneRenderer::setDefaultPointSize(double size)
{
    if(size != impl->defaultPointSize){
        impl->defaultPointSize = size;
        requestToClearCache();
    }
}


void GLSceneRenderer::setDefaultLineWidth(double width)
{
    if(width != impl->defaultLineWidth){
        impl->defaultLineWidth = width;
        requestToClearCache();
    }
}


void GLSceneRenderer::showNormalVectors(double length)
{
    bool doNormalVisualization = (length > 0.0);
    if(doNormalVisualization != impl->doNormalVisualization || length != impl->normalLength){
        impl->doNormalVisualization = doNormalVisualization;
        impl->normalLength = length;
        requestToClearCache();
    }
}


void GLSceneRenderer::setNewDisplayListDoubleRenderingEnabled(bool on)
{
    impl->isNewDisplayListDoubleRenderingEnabled = on;
}


void GLSceneRenderer::enableUnusedCacheCheck(bool on)
{
    if(!on){
        impl->nextCacheMap->clear();
    }
    impl->doUnusedCacheCheck = on;
}


const Affine3& GLSceneRenderer::currentModelTransform() const
{
    impl->tmpCurrentModelTransform = impl->lastViewMatrix.inverse() * impl->Vstack.back();
    return impl->tmpCurrentModelTransform;
}


const Affine3& GLSceneRenderer::currentCameraPosition() const
{
    impl->tmpCurrentCameraPosition = impl->lastViewMatrix.inverse();
    return impl->tmpCurrentCameraPosition;
}


const Matrix4& GLSceneRenderer::projectionMatrix() const
{
    return impl->lastProjectionMatrix;
}


void GLSceneRenderer::getViewFrustum
(const SgPerspectiveCamera& camera, double& left, double& right, double& bottom, double& top) const
{
    top = camera.nearDistance() * tan(camera.fovy(impl->aspectRatio) / 2.0);
    bottom = -top;
    right = top * impl->aspectRatio;
    left = -right;
}


void GLSceneRenderer::getViewVolume
(const SgOrthographicCamera& camera, double& out_left, double& out_right, double& out_bottom, double& out_top) const
{
    GLfloat left, right, bottom, top;
    impl->getViewVolume(camera, left, right, bottom, top);
    out_left = left;
    out_right = right;
    out_bottom = bottom;
    out_top = top;
}


void GLSceneRenderer::visitOutlineGroup(SgOutlineGroup* outline)
{
    impl->visitOutlineGroup(outline);
}


void GLSceneRendererImpl::visitOutlineGroup(SgOutlineGroup* outlineGroup)
{
    glClearStencil(0);
    glClear(GL_STENCIL_BUFFER_BIT);

    glEnable(GL_STENCIL_TEST);

    glStencilFunc(GL_ALWAYS, 1, -1);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

    for(SgGroup::const_iterator p = outlineGroup->begin(); p != outlineGroup->end(); ++p){
        (*p)->accept(*self);
    }

    glStencilFunc(GL_NOTEQUAL, 1, -1);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

    glPushAttrib(GL_POLYGON_BIT);
    glLineWidth(outlineGroup->lineWidth()*2+1);
    glPolygonMode(GL_FRONT, GL_LINE);
    setColor(outlineGroup->color());
    enableColorMaterial(true);
    for(SgGroup::const_iterator p = outlineGroup->begin(); p != outlineGroup->end(); ++p){
        (*p)->accept(*self);
    }
    enableColorMaterial(false);
    setLineWidth(lineWidth);
    glPopAttrib();

    glDisable(GL_STENCIL_TEST);

}
